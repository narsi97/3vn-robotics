# Copyright 2026 3VN Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Shared fixtures for the robot-description tests.

Every test expands the SAME xacro entry point, with the same arguments,
that `ros2 launch` uses at runtime. That is deliberate: a test that built
its own URDF a different way would be testing itself rather than the
thing that actually runs.
"""
import pathlib

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

PKG = pathlib.Path(get_package_share_directory('threevn_robot_description'))
CONFIG_DIR = PKG / 'config'

#: Entry point per robot family. A profile is expanded through the xacro
#: belonging to its `meta.kind` -- which is the whole reason that key
#: exists. Before it did, the suite globbed every YAML in config/ and fed
#: it to the ARM xacro, so adding the mobile base produced 25 failures
#: that said nothing about the base and everything about the harness.
ENTRY_POINTS = {
    'arm': PKG / 'urdf' / 'threevn_arm.urdf.xacro',
    'base': PKG / 'urdf' / 'threevn_base.urdf.xacro',
    'mobile_manipulator': PKG / 'urdf' / 'threevn_mobile_manipulator.urdf.xacro',
}

#: Plausible total mass per family, kg. An order-of-magnitude mass error
#: is the most common URDF bug and the one that makes a simulation behave
#: bizarrely rather than fail outright.
MASS_BAND = {
    'arm': (0.2, 3.0),
    'base': (0.5, 6.0),
    # The composed robot is the sum of the two, so its band is theirs
    # added. A composition that lost or double-counted a subsystem's mass
    # would land outside it.
    'mobile_manipulator': (0.7, 9.0),
}

#: Frames each family must expose. For the base these are the two that
#: Phase 11 composes against; losing either silently breaks mounting.
REQUIRED_FRAMES = {
    'arm': {
        'base_footprint', 'base_link', 'tool0', 'grasp_frame',
        'camera_mount_link', 'camera_link', 'camera_optical_frame',
        'imu_link',
    },
    'base': {'base_footprint', 'base_link', 'arm_mount_link'},
    # The composed robot must expose BOTH interfaces: the base's
    # navigation frames unprefixed, and the arm's TCP under its prefix.
    'mobile_manipulator': {
        'base_footprint', 'base_link', 'arm_mount_link',
        'arm_base_link', 'arm_tool0', 'arm_grasp_frame',
        'arm_camera_optical_frame',
    },
}


def profile_kind(profile):
    """Return the robot family a profile declares."""
    return yaml.safe_load(pathlib.Path(profile).read_text())['meta']['kind']


def component_paths(profile):
    """
    Return the component profiles a composition is built from.

    A component profile is its own only component, which lets callers
    treat both kinds uniformly instead of branching on the family.
    """
    doc = yaml.safe_load(pathlib.Path(profile).read_text())
    parts = doc.get('components')
    if not parts:
        return [pathlib.Path(profile)]
    return [CONFIG_DIR / name for name in parts.values()]


def min_inertia_for(profile):
    """
    Return the inertia floor that applies to a profile's links.

    A composition declares no `defaults` of its own -- each component
    brings its own floor, and the composed robot has to satisfy the
    lowest of them, because that is the weakest guarantee any of its
    links was built under.
    """
    floors = [
        yaml.safe_load(part.read_text())['defaults']['min_inertia']
        for part in component_paths(profile)
    ]
    return min(floors)


#: Every profile shipped in the package, regardless of family. Tests whose
#: assertions are genuinely generic -- inertia algebra, structural sanity,
#: the hardware seam -- parametrize over THIS, so a new robot inherits them
#: for free.
ALL_PROFILES = sorted(CONFIG_DIR.glob('threevn_*.yaml'))

PROFILES_BY_KIND = {}
for _p in ALL_PROFILES:
    PROFILES_BY_KIND.setdefault(profile_kind(_p), []).append(_p)

ARM_PROFILES = PROFILES_BY_KIND.get('arm', [])
BASE_PROFILES = PROFILES_BY_KIND.get('base', [])

#: Families that describe hardware directly, declaring links, joints and
#: masses. A COMPOSITION profile (the mobile manipulator) names component
#: profiles and where they meet, and deliberately declares none of those
#: things -- so the component schema tests do not apply to it, and
#: running them against it fails on missing keys rather than on anything
#: real.
COMPONENT_KINDS = {'arm', 'base'}

COMPONENT_PROFILES = [p for p in ALL_PROFILES
                      if profile_kind(p) in COMPONENT_KINDS]
COMPOSED_PROFILES = [p for p in ALL_PROFILES
                     if profile_kind(p) not in COMPONENT_KINDS]

#: Arm-family profiles. Tests that read arm-only keys (`servos`, `frames`,
#: `initial_position_deg`) use this. The base's equivalents are asserted in
#: test_base_description.py, against `motors` and continuous wheel joints.
PROFILES = ARM_PROFILES

DEFAULT_PROFILE = CONFIG_DIR / 'threevn_arm_v1.yaml'
XACRO_ENTRY = ENTRY_POINTS['arm']

#: The three hardware targets. `esp32` has no implementation until Phase 5,
#: but the DESCRIPTION must already expand for it -- that is what makes the
#: seam real rather than aspirational.
TARGETS = ['mock', 'gz', 'esp32']

#: Plugin each target must select. Duplicated from the xacro on purpose:
#: if someone edits one, the test fails rather than silently agreeing.
EXPECTED_PLUGIN = {
    'mock': 'mock_components/GenericSystem',
    'gz': 'gz_ros2_control/GazeboSimSystem',
    'esp32': 'threevn_hardware/Esp32SystemInterface',
}


@pytest.fixture(scope='session')
def expanded():
    """
    Expand the xacro for a profile, memoised per (profile, target).

    The entry point follows the profile's family, so a test parametrized
    over ALL_PROFILES expands each one through its own xacro without
    knowing which robot it is looking at.

    Expansion is the slow part of these tests; caching keeps the whole
    suite inside its 60-second budget.
    """
    cache = {}

    def _expand(profile=DEFAULT_PROFILE, target='mock', prefix='',
                controllers_file=''):
        key = (str(profile), target, prefix, controllers_file)
        if key not in cache:
            doc = xacro.process_file(
                str(ENTRY_POINTS[profile_kind(profile)]),
                mappings={
                    'params_file': str(profile),
                    'target': target,
                    'prefix': prefix,
                    'controllers_file': controllers_file,
                },
            )
            cache[key] = doc.toprettyxml(indent='  ')
        return cache[key]

    return _expand

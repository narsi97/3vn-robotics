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
No joint is configured beyond what its servo can physically deliver.

This exists because of a real defect. The shoulder was configured with
`effort: 1.5` N.m while an MG996R delivers 0.922 N.m at 4.8 V. Nothing
failed: Gazebo happily simulated motions the physical arm would stall on,
which is precisely the sim-to-real mismatch this platform is meant to
catch rather than create.

A corrected number alone would not stop it recurring. These assertions do.
"""

from conftest import PROFILES
import pytest
from urdf_parser_py.urdf import URDF
import yaml


def _cfg(profile):
    return yaml.safe_load(profile.read_text())


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_every_actuated_joint_names_a_servo(profile):
    cfg = _cfg(profile)
    for name, spec in cfg['joints'].items():
        if spec['type'] == 'fixed' or 'mimic' in spec:
            continue
        assert 'servo' in spec, f'{name}: no servo declared'
        assert spec['servo'] in cfg['servos'], \
            f'{name}: unknown servo {spec["servo"]!r}'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_revolute_effort_within_servo_stall_torque(profile):
    """
    A revolute joint may not be configured above its servo's stall torque.

    Stall torque is the absolute ceiling: a servo held there overheats and
    strips its gears, so it is a limit, never a design point.
    """
    cfg = _cfg(profile)
    derate = cfg['limit_derate']['effort']
    for name, spec in cfg['joints'].items():
        if spec['type'] != 'revolute':
            continue
        servo = cfg['servos'][spec['servo']]
        ceiling = servo['stall_torque_nm'] * derate
        assert spec['limit']['effort'] <= ceiling + 1e-9, (
            f'{name}: effort {spec["limit"]["effort"]} N.m exceeds what a '
            f'{spec["servo"]} can deliver ({ceiling} N.m at '
            f'{servo["voltage"]} V). Gazebo would validate motions that '
            'stall on the real arm.'
        )


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_prismatic_force_within_servo_capability(profile):
    """
    Prismatic effort is a force, so it is checked through the horn radius.

    torque = force x radius, so a servo driving a finger through a horn of
    radius r can produce at most stall_torque / r newtons.
    """
    cfg = _cfg(profile)
    for name, spec in cfg['joints'].items():
        if spec['type'] != 'prismatic' or 'mimic' in spec:
            continue
        servo = cfg['servos'][spec['servo']]
        radius = spec['servo_horn_radius_m']
        ceiling = servo['stall_torque_nm'] / radius
        assert spec['limit']['effort'] <= ceiling + 1e-9, (
            f'{name}: {spec["limit"]["effort"]} N exceeds the '
            f'{ceiling:.1f} N a {spec["servo"]} can produce through a '
            f'{radius * 1000:.0f} mm horn.'
        )


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_revolute_velocity_respects_derate(profile):
    """
    Joint velocity stays below the derated no-load servo speed.

    Datasheet speed is measured with no load. A servo moving an arm is
    substantially slower, so configuring the no-load figure would let the
    planner generate trajectories the hardware cannot track.
    """
    cfg = _cfg(profile)
    derate = cfg['limit_derate']['velocity']
    for name, spec in cfg['joints'].items():
        if spec['type'] != 'revolute':
            continue
        servo = cfg['servos'][spec['servo']]
        ceiling = servo['no_load_speed_rads'] * derate
        assert spec['limit']['velocity'] <= ceiling + 1e-9, (
            f'{name}: velocity {spec["limit"]["velocity"]} rad/s exceeds '
            f'{ceiling:.2f} rad/s ({derate:.0%} of a {spec["servo"]}\'s '
            'no-load speed).'
        )


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_urdf_carries_the_servo_derived_limits(expanded, profile):
    """The derived values must actually reach the URDF, not just the YAML."""
    cfg = _cfg(profile)
    robot = URDF.from_xml_string(expanded(profile=profile))
    for name, spec in cfg['joints'].items():
        if spec['type'] == 'fixed':
            continue
        joint = robot.joint_map[name]
        assert joint.limit.effort == pytest.approx(spec['limit']['effort'])


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_servo_entries_declare_datasheet_provenance(profile):
    """
    Servo figures must come from a datasheet, never an estimate.

    Unlike link masses, which are legitimately estimated until an arm is
    built, a servo's rating is published. An estimate here would mean
    nobody looked it up.
    """
    for name, servo in _cfg(profile)['servos'].items():
        assert servo.get('provenance') == 'datasheet', \
            f'servo {name}: provenance must be "datasheet"'
        assert servo['stall_torque_nm'] > 0
        assert servo['no_load_speed_rads'] > 0

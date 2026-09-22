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

PKG = pathlib.Path(get_package_share_directory('threevn_robot_description'))
XACRO_ENTRY = PKG / 'urdf' / 'threevn_arm.urdf.xacro'
CONFIG_DIR = PKG / 'config'
DEFAULT_PROFILE = CONFIG_DIR / 'threevn_arm_v1.yaml'

#: Every robot profile shipped in the package. Tests parametrize over all
#: of them so adding a profile automatically widens the test matrix.
PROFILES = sorted(CONFIG_DIR.glob('threevn_*.yaml'))

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
    Expand the xacro, memoised per (profile, target).

    Expansion is the slow part of these tests; caching keeps the whole
    suite inside its 60-second budget.
    """
    cache = {}

    def _expand(profile=DEFAULT_PROFILE, target='mock', prefix=''):
        key = (str(profile), target, prefix)
        if key not in cache:
            doc = xacro.process_file(
                str(XACRO_ENTRY),
                mappings={
                    'params_file': str(profile),
                    'target': target,
                    'prefix': prefix,
                },
            )
            cache[key] = doc.toprettyxml(indent='  ')
        return cache[key]

    return _expand

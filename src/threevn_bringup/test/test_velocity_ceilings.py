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
The controller may not promise speeds the motors cannot deliver.

Two files have to agree and nothing forced them to: base_controllers.yaml
sets ceilings in m/s and rad/s, while threevn_base_v1.yaml limits each
wheel in rad/s. The conversion between them runs through the wheel radius
AND the scrub multiplier, so a plausible-looking number in either file
can quietly exceed the motor.

It already had: the linear ceiling shipped at 0.65 m/s, which needs
0.65 / 0.033 = 19.70 rad/s from wheels limited to 19.0. Nothing
complained, because nothing compared them. Under load the joint limiter
simply clamps, so the robot is slower than commanded and the logs say
nothing -- the same silent class as the arm's over-torqued gripper.
"""

import pathlib

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

BRINGUP = pathlib.Path(get_package_share_directory('threevn_bringup'))
DESCRIPTION = pathlib.Path(
    get_package_share_directory('threevn_robot_description'))

CONTROLLERS = BRINGUP / 'config' / 'base_controllers.yaml'
BASE_PROFILE = DESCRIPTION / 'config' / 'threevn_base_v1.yaml'


@pytest.fixture(scope='module')
def params():
    """Provide the diff_drive_controller parameter block."""
    doc = yaml.safe_load(CONTROLLERS.read_text())
    return doc['base_controller']['ros__parameters']


@pytest.fixture(scope='module')
def wheel_limit():
    """Provide the per-wheel velocity limit, in rad/s."""
    cfg = yaml.safe_load(BASE_PROFILE.read_text())
    limits = {name: spec['limit']['velocity']
              for name, spec in cfg['joints'].items()}
    # All four wheels share a limit; if they ever diverge, the tightest
    # one is what the robot can actually do.
    return min(limits.values())


def wheel_speed_for(params, linear, angular):
    """
    Return the wheel speed a (linear, angular) command demands, rad/s.

    This is diff_drive_controller's inverse kinematics: the scrub
    multiplier widens the track the controller believes it has, so
    correcting odometry makes turning MORE demanding on the motors.
    """
    radius = params['wheel_radius'] * params['left_wheel_radius_multiplier']
    separation = (params['wheel_separation']
                  * params['wheel_separation_multiplier'])
    return abs(linear) / radius + abs(angular) * separation / 2.0 / radius


def test_linear_ceiling_is_achievable(params, wheel_limit):
    """Driving straight at the configured top speed must be possible."""
    top = params['linear']['x']['max_velocity']
    needed = wheel_speed_for(params, top, 0.0)
    assert needed <= wheel_limit, (
        f'max_velocity {top} m/s needs {needed:.2f} rad/s per wheel, '
        f'above the {wheel_limit} rad/s limit. Highest achievable is '
        f'{wheel_limit * params["wheel_radius"]:.3f} m/s.'
    )


def test_reverse_ceiling_is_achievable(params, wheel_limit):
    """Reverse has its own parameter, and it has been missed before."""
    bottom = params['linear']['x']['min_velocity']
    needed = wheel_speed_for(params, bottom, 0.0)
    assert needed <= wheel_limit, (
        f'min_velocity {bottom} m/s needs {needed:.2f} rad/s per wheel, '
        f'above the {wheel_limit} rad/s limit'
    )


def test_angular_ceiling_is_achievable(params, wheel_limit):
    """
    Turning in place at the configured rate must be possible.

    This is the one the scrub calibration threatens: raising
    wheel_separation_multiplier to correct heading multiplies the wheel
    speed a turn demands. At 1.0 the angular ceiling was comfortable; at
    2.40 it is close to the motor limit, and raising it further would
    silently make the top turn rate unreachable.
    """
    top = params['angular']['z']['max_velocity']
    needed = wheel_speed_for(params, 0.0, top)
    assert needed <= wheel_limit, (
        f'angular max_velocity {top} rad/s needs {needed:.2f} rad/s per '
        f'wheel, above the {wheel_limit} rad/s limit. This is what the '
        f'scrub multiplier ({params["wheel_separation_multiplier"]}) '
        f'costs.'
    )


def test_simultaneous_maxima_are_known_to_saturate(params, wheel_limit):
    """
    Full speed AND full turn together is deliberately NOT achievable.

    Documenting rather than fixing. Requiring it would force the straight
    line speed down to roughly half the motor's capability to buy a
    combination the robot has no reason to command. The joint limiter
    clamps instead, which distorts the path rather than damaging
    anything.

    The test exists so the trade-off stays a decision. If it ever starts
    failing, someone has widened the margins and should confirm that was
    intended.
    """
    needed = wheel_speed_for(params,
                             params['linear']['x']['max_velocity'],
                             params['angular']['z']['max_velocity'])
    assert needed > wheel_limit, (
        'both axes at maximum now fit within the wheel limit. That is not '
        'a failure, but it means the margins changed -- update this test.'
    )

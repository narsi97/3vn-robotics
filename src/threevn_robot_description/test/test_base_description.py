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
The mobile base description. No simulator, no robot.

Same guarantees the arm description already has, plus the ones specific
to a thing with wheels: the geometry must actually reach the ground, the
wheels must turn about the right axis, and the base must remain
independent of the arm so Phase 11 can compose them rather than merge
them.
"""

import math
import pathlib

from ament_index_python.packages import get_package_share_directory
import pytest
from urdf_parser_py.urdf import URDF
import xacro
import yaml

SHARE = pathlib.Path(get_package_share_directory('threevn_robot_description'))
BASE_XACRO = SHARE / 'urdf' / 'threevn_base.urdf.xacro'
BASE_PROFILE = SHARE / 'config' / 'threevn_base_v1.yaml'

WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint']

TARGETS = ['mock', 'gz', 'esp32']

EXPECTED_PLUGIN = {
    'mock': 'mock_components/GenericSystem',
    'gz': 'gz_ros2_control/GazeboSimSystem',
    'esp32': 'threevn_hardware/Esp32SystemInterface',
}


@pytest.fixture(scope='module')
def cfg():
    """Provide the base profile."""
    return yaml.safe_load(BASE_PROFILE.read_text())


@pytest.fixture(scope='module')
def expanded():
    """Expand the base description, memoised per target."""
    cache = {}

    def _expand(target='mock'):
        if target not in cache:
            cache[target] = xacro.process_file(
                str(BASE_XACRO),
                mappings={'params_file': str(BASE_PROFILE),
                          'target': target, 'prefix': '',
                          'controllers_file': ''},
            ).toprettyxml(indent='  ')
        return cache[target]

    return _expand


# -- structure ---------------------------------------------------------

@pytest.mark.parametrize('target', TARGETS)
def test_expands_for_every_target(expanded, target):
    assert '<robot' in expanded(target=target)


def test_all_four_wheels_exist(expanded):
    robot = URDF.from_xml_string(expanded())
    for wheel in WHEELS:
        assert wheel in robot.joint_map, f'{wheel} missing'


def test_wheels_are_continuous_not_revolute(expanded):
    """
    A wheel has no travel limit.

    Declaring one makes the controller fight the joint after a few
    metres, which presents as a base that gradually stops for no visible
    reason.
    """
    robot = URDF.from_xml_string(expanded())
    for wheel in WHEELS:
        assert robot.joint_map[wheel].type == 'continuous', \
            f'{wheel} is {robot.joint_map[wheel].type}, should be continuous'


def test_wheels_turn_about_y(expanded):
    """
    All four roll about the same axis.

    A wheel with the wrong axis still spins, so the simulation looks
    plausible while the robot crabs sideways.
    """
    robot = URDF.from_xml_string(expanded())
    for wheel in WHEELS:
        axis = robot.joint_map[wheel].axis
        assert [abs(round(a)) for a in axis] == [0, 1, 0], \
            f'{wheel} axis is {axis}'


def test_the_chassis_sits_on_its_wheels(expanded, cfg):
    """
    Geometry must reach the ground.

    Getting the ride height wrong buries the robot in the floor or floats
    it, and both make the physics behave in ways that read as a
    controller bug rather than a description one.
    """
    robot = URDF.from_xml_string(expanded())
    radius = cfg['links']['wheel_link']['geometry']['radius']

    chassis_z = robot.joint_map['chassis_joint'].origin.position[2]
    wheel_z = robot.joint_map[WHEELS[0]].origin.position[2]
    axle_height = chassis_z + wheel_z

    assert axle_height == pytest.approx(radius, abs=1e-6), (
        f'wheel axles sit at {axle_height:.4f} m but the wheel radius is '
        f'{radius:.4f} m -- the base would be {"floating" if axle_height > radius else "buried"}'
    )


def test_wheels_are_symmetric_about_the_centreline(expanded):
    """An asymmetric chassis drifts when commanded straight."""
    robot = URDF.from_xml_string(expanded())
    left = robot.joint_map['front_left_wheel_joint'].origin.position
    right = robot.joint_map['front_right_wheel_joint'].origin.position
    assert left[0] == pytest.approx(right[0])
    assert left[1] == pytest.approx(-right[1])


def test_wheel_separation_matches_the_actual_geometry(expanded, cfg):
    """
    The declared separation must match where the wheels really are.

    diff_drive_controller converts a twist into wheel speeds using this
    number. If it disagrees with the model, every commanded rotation is
    wrong by a constant factor and the odometry is wrong with it.
    """
    robot = URDF.from_xml_string(expanded())
    left = robot.joint_map['front_left_wheel_joint'].origin.position[1]
    right = robot.joint_map['front_right_wheel_joint'].origin.position[1]
    assert abs(left - right) == pytest.approx(
        cfg['geometry']['wheel_separation'], abs=1e-6)


def test_single_root_and_no_orphans(expanded):
    robot = URDF.from_xml_string(expanded())
    children = {j.child for j in robot.joints}
    roots = [link.name for link in robot.links if link.name not in children]
    assert roots == ['base_footprint'], f'expected one root, got {roots}'


def test_inertia_is_physically_valid(expanded):
    """Same check the arm gets: an invalid tensor produces NaN on contact."""
    import numpy as np
    for link in URDF.from_xml_string(expanded()).links:
        if link.inertial is None:
            continue
        i = link.inertial.inertia
        tensor = np.array([[i.ixx, i.ixy, i.ixz],
                           [i.ixy, i.iyy, i.iyz],
                           [i.ixz, i.iyz, i.izz]])
        eigenvalues = np.linalg.eigvalsh(tensor)
        assert (eigenvalues > 0).all(), f'{link.name}: not positive definite'
        a, b, c = sorted(eigenvalues)
        assert a + b >= c * (1 - 1e-9), f'{link.name}: triangle inequality'


# -- the hardware seam, same rules as the arm --------------------------

@pytest.mark.parametrize('target', TARGETS)
def test_exactly_one_plugin_per_target(expanded, target):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(expanded(target=target))
    plugins = [p.text.strip() for rc in root.iter('ros2_control')
               for p in rc.iter('plugin') if p.text]
    assert plugins == [EXPECTED_PLUGIN[target]]


@pytest.mark.parametrize('target', ['mock', 'esp32'])
def test_non_sim_targets_reference_no_gazebo(expanded, target):
    """The architecture rule, applied to the base as well as the arm."""
    xml = expanded(target=target)
    assert 'gz_ros2_control' not in xml
    assert '<gazebo' not in xml


def test_wheels_are_velocity_commanded_not_position(expanded):
    """
    You do not tell a wheel where to be.

    This is why the base uses diff_drive_controller rather than the arm's
    trajectory controller, and a position interface here would be a
    category error that only shows up at runtime.
    """
    import xml.etree.ElementTree as ET
    rc = next(ET.fromstring(expanded()).iter('ros2_control'))
    for joint in rc.iter('joint'):
        names = [c.get('name') for c in joint.findall('command_interface')]
        assert names == ['velocity'], f'{joint.get("name")}: {names}'


# -- independence from the arm ----------------------------------------

def test_the_base_does_not_depend_on_the_arm(expanded):
    """
    The base is a complete robot on its own.

    Phase 11 composes the two; it does not merge them. If the base
    description referenced arm links, they could not be developed,
    tested or deployed separately.
    """
    xml = expanded()
    for arm_link in ('shoulder_pan_joint', 'gripper_base_link', 'tool0'):
        assert arm_link not in xml, f'the base references {arm_link}'


def test_the_base_offers_a_mount_point_for_the_arm(expanded):
    """
    Declared now, unused today.

    Costs one fixed joint and turns "mount the arm" in Phase 11 into a
    one-joint change rather than a description rewrite - the same bet
    base_footprint and mount_plate_link made in Phase 1.
    """
    robot = URDF.from_xml_string(expanded())
    assert 'arm_mount_link' in {link.name for link in robot.links}


# -- motor limits, same pattern as the arm's servos --------------------

def test_wheel_limits_stay_within_the_motor(cfg):
    """
    A joint may not be configured beyond what its motor can deliver.

    Same rule as the arm's servos: a velocity limit above what the
    gearmotor can reach lets a planner generate trajectories the hardware
    cannot track.
    """
    motor = cfg['motors']['tt_gearmotor']
    ceiling = motor['no_load_speed_rads'] * cfg['limit_derate']['velocity']
    for name, joint in cfg['joints'].items():
        assert joint['limit']['velocity'] <= ceiling + 1e-9, (
            f'{name}: {joint["limit"]["velocity"]} rad/s exceeds '
            f'{ceiling:.2f} rad/s for a {list(cfg["motors"])[0]}')


def test_motor_figures_declare_datasheet_provenance(cfg):
    for name, motor in cfg['motors'].items():
        assert motor['provenance'] == 'datasheet', f'{name}'


def test_the_drive_model_is_recorded(cfg):
    """
    Skid steer has real consequences for odometry.

    Recording it in the profile means the limitation travels with the
    robot rather than living only in a comment somebody deletes.
    """
    assert cfg['meta']['drive'] == 'skid_steer'
    assert 'wheel_separation_multiplier' in cfg['odometry']


def test_max_linear_speed_is_achievable(cfg):
    """
    The configured top speed must be reachable by the wheels.

    v = omega * r. A controller limit above this is a number the robot
    can never reach, which makes every velocity command subtly wrong.
    """
    motor = cfg['motors']['tt_gearmotor']
    radius = cfg['links']['wheel_link']['geometry']['radius']
    top = motor['no_load_speed_rads'] * radius
    assert top > 0.5, f'top speed {top:.2f} m/s is implausibly slow'
    assert top < 2.0, f'top speed {top:.2f} m/s is implausibly fast for a desk robot'
    assert math.isclose(top, 0.693, abs_tol=0.01), (
        f'top speed {top:.3f} m/s; base_controllers.yaml caps linear.x at '
        '0.65, which must stay below it'
    )

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
A profile and its controllers file name the same joints.

Two files have to agree and nothing forced them to. The profile lists
the arm's joints and the gripper's; the controllers file lists the
joints arm_controller and gripper_controller drive. They live in
different packages because they are different kinds of fact, and they
are joined only at launch.

When they disagree the failure is far from the cause: the controller
either fails to claim an interface, or rejects a trajectory naming a
joint it does not have -- "goal rejected", with no hint that a name is
at fault.
"""
import pathlib

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

BRINGUP = pathlib.Path(get_package_share_directory('threevn_bringup'))
DESCRIPTION = pathlib.Path(
    get_package_share_directory('threevn_robot_description'))


def _robots():
    """Return (profile path, controllers path) for every single-arm profile."""
    pairs = []
    for profile in sorted((DESCRIPTION / 'config').glob('*.yaml')):
        cfg = yaml.safe_load(profile.read_text())
        if 'arm' not in cfg or 'gripper' not in cfg:
            # A base, or a composition: it has no arm block of its own,
            # and its controllers are merged from its components.
            continue
        name = (cfg.get('control') or {}).get('controllers', 'controllers.yaml')
        pairs.append(pytest.param(profile, BRINGUP / 'config' / name,
                                  id=profile.stem))
    return pairs


ROBOTS = _robots()


def test_there_is_something_to_compare():
    """
    At least the 3VN arm and the SO-101 are found.

    Guards the tests below against passing because the glob matched
    nothing.
    """
    names = {param.id for param in ROBOTS}
    assert {'threevn_arm_v1', 'so101_follower'} <= names


@pytest.mark.parametrize('profile,controllers', ROBOTS)
def test_the_controllers_file_exists(profile, controllers):
    """A profile cannot name a controllers file that is not installed."""
    assert controllers.is_file(), f'{profile.name} names {controllers.name}'


@pytest.mark.parametrize('profile,controllers', ROBOTS)
def test_arm_controller_drives_the_profiles_arm_joints(profile, controllers):
    """Same joints, same ORDER: a pose is a list, and order is its meaning."""
    cfg = yaml.safe_load(profile.read_text())
    params = yaml.safe_load(controllers.read_text())
    assert params['arm_controller']['ros__parameters']['joints'] == \
        cfg['arm']['joints']


@pytest.mark.parametrize('profile,controllers', ROBOTS)
def test_gripper_controller_drives_the_profiles_gripper_joint(
        profile, controllers):
    """The joint open_gripper() commands is the one the profile measured."""
    cfg = yaml.safe_load(profile.read_text())
    params = yaml.safe_load(controllers.read_text())
    assert params['gripper_controller']['ros__parameters']['joint'] == \
        cfg['gripper']['joint']


@pytest.mark.parametrize('profile,controllers', ROBOTS)
def test_every_robot_exposes_the_same_three_controllers(profile, controllers):
    """
    The controller NAMES and TYPES are identical across robots.

    They are the public API: /arm_controller/follow_joint_trajectory
    and /gripper_controller/gripper_cmd are what a scenario calls, on
    any robot. A robot that renamed one would need its own client.
    """
    manager = yaml.safe_load(controllers.read_text())[
        'controller_manager']['ros__parameters']
    assert manager['arm_controller']['type'] == \
        'joint_trajectory_controller/JointTrajectoryController'
    assert manager['gripper_controller']['type'] == \
        'position_controllers/GripperActionController'
    assert manager['joint_state_broadcaster']['type'] == \
        'joint_state_broadcaster/JointStateBroadcaster'
    assert manager['enforce_command_limits'] is True

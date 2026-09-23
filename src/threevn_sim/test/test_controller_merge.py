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
The composed robot's controller parameters, merged at launch.

Phase 11 chose to merge the two subsystems' controller configs when the
launch runs rather than commit a third YAML, so that adding a joint to
the arm cannot leave the composed robot driving a stale list. That put
real logic in a launch file, and logic that runs only when a simulator
starts is logic nobody tests.

These run in milliseconds against no ROS graph at all.
"""

import importlib.util
import pathlib

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

LAUNCH = (pathlib.Path(get_package_share_directory('threevn_sim'))
          / 'launch' / 'mm_sim.launch.py')
BRINGUP = (pathlib.Path(get_package_share_directory('threevn_bringup'))
           / 'config')


def _module():
    """Import the launch file as a module, for its pure functions."""
    spec = importlib.util.spec_from_file_location('mm_sim_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def mm_sim():
    """Provide the launch module."""
    return _module()


@pytest.fixture(scope='module')
def arm_params():
    """Provide the arm's controller config, as shipped."""
    return yaml.safe_load((BRINGUP / 'controllers.yaml').read_text())


@pytest.fixture(scope='module')
def base_params():
    """Provide the base's controller config, as shipped."""
    return yaml.safe_load((BRINGUP / 'base_controllers.yaml').read_text())


def test_arm_joints_gain_the_prefix(mm_sim, arm_params):
    """Every arm joint name is prefixed, because the composition renames them."""
    out = mm_sim.prefix_joint_names(arm_params, 'arm_')
    joints = out['arm_controller']['ros__parameters']['joints']
    assert joints == ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint',
                      'arm_elbow_joint', 'arm_wrist_joint']


def test_a_scalar_joint_name_is_prefixed_too(mm_sim, arm_params):
    """
    The gripper names ONE joint, as a string rather than a list.

    A merge that only handled lists would leave the gripper pointing at a
    joint that does not exist on the composed robot, and the failure
    arrives at controller activation rather than here.
    """
    out = mm_sim.prefix_joint_names(arm_params, 'arm_')
    assert out['gripper_controller']['ros__parameters']['joint'] == \
        'arm_gripper_left_finger_joint'


def test_controller_names_are_not_prefixed(mm_sim, arm_params):
    """
    The controllers keep their names, so the interface does not move.

    /arm_controller/follow_joint_trajectory is the same action on the
    composed robot as on the arm alone. Prefixing controller names as
    well would rename every topic and action, which is exactly the
    breakage composition is supposed to avoid.
    """
    out = mm_sim.prefix_joint_names(arm_params, 'arm_')
    assert 'arm_controller' in out
    assert 'gripper_controller' in out
    assert 'arm_arm_controller' not in out
    types = out['controller_manager']['ros__parameters']
    assert 'arm_controller' in types
    assert 'gripper_controller' in types


def test_the_original_is_not_modified(mm_sim, arm_params):
    """The shipped config is a fixture here and input everywhere else."""
    before = yaml.safe_dump(arm_params)
    mm_sim.prefix_joint_names(arm_params, 'arm_')
    assert yaml.safe_dump(arm_params) == before


def test_the_merge_keeps_every_controller(mm_sim, arm_params, base_params):
    """One controller_manager hosts all four controllers."""
    merged = mm_sim.merge_controller_params(arm_params, base_params, 'arm_')
    hosted = merged['controller_manager']['ros__parameters']
    for name in ('joint_state_broadcaster', 'arm_controller',
                 'gripper_controller', 'base_controller'):
        assert name in hosted, f'{name} was lost in the merge'
    assert 'base_controller' in merged
    assert 'arm_controller' in merged


def test_the_wheels_are_not_prefixed(mm_sim, arm_params, base_params):
    """
    The base keeps the unprefixed names it already had.

    In the composed robot the base IS the robot, so only the arm moves
    into a namespace. Prefixing the wheels as well would have been
    symmetrical and wrong.
    """
    merged = mm_sim.merge_controller_params(arm_params, base_params, 'arm_')
    wheels = merged['base_controller']['ros__parameters']['left_wheel_names']
    assert wheels == ['front_left_wheel_joint', 'rear_left_wheel_joint']


def test_a_disagreement_between_subsystems_is_refused(mm_sim, arm_params,
                                                      base_params):
    """
    The two configs must AGREE on controller_manager, not overwrite.

    Both declare update_rate. Letting one silently win would mean the
    composed robot ran at a rate neither config states, and nothing would
    say so -- the arm's tuning would simply stop matching its loop.
    """
    import copy

    clashing = copy.deepcopy(base_params)
    clashing['controller_manager']['ros__parameters']['update_rate'] = 25

    with pytest.raises(ValueError, match='update_rate'):
        mm_sim.merge_controller_params(arm_params, clashing, 'arm_')


def test_the_shipped_configs_actually_agree(mm_sim, arm_params, base_params):
    """
    And they do agree today, which is what makes the check above cheap.

    If this ever fails, the two subsystems have drifted apart and the
    composed robot cannot run both until someone decides which rate is
    right.
    """
    mm_sim.merge_controller_params(arm_params, base_params, 'arm_')

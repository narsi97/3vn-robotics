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
The client takes a robot's facts from the robot. No robot, no ROS.

Three things used to be constants in the client and are now read from
the description the robot is running: the named poses, the gripper's
travel, and which joint the gripper is. These tests cover the reading,
and then run the whole scenario suite against two stand-in robots that
share nothing -- a different number of joints, different names, one
gripper in metres with a mimic finger and one in radians without.

A stand-in cannot prove the real client works; `make test-ros` does
that, against a real graph, for whichever PROFILE it is given. What it
proves is narrower and worth having in the fast suite: that no scenario
names a joint, a pose value or a gripper width of its own.
"""

import math
import types
import xml.etree.ElementTree as ET

import pytest
from threevn_control.robot_client import LimitViolation, RobotClient
from threevn_control.scenarios import arm  # noqa: F401  (registers scenarios)
from threevn_control.scenarios.base import REGISTRY

DESCRIPTION = """
<robot name="five">
  <joint name="a" type="revolute"><limit lower="-1" upper="1"/></joint>
  <joint name="jaw" type="revolute"><limit lower="-0.1" upper="1.7"/></joint>
  <joint name="jaw_mirror" type="revolute">
    <mimic joint="jaw" multiplier="1.0"/>
  </joint>
  <threevn_semantics profile="five_v1">
    <arm joints="a b c" base_frame="base" tool_frame="tip"/>
    <gripper joint="jaw" open="1.4" closed="0.0"/>
    <pose name="home" positions="0.0 0.0 0.0"/>
    <pose name="ready" positions="0.1 0.2 0.3"/>
  </threevn_semantics>
</robot>
"""


# -- reading the description -------------------------------------------

def test_semantics_are_parsed_from_the_description():
    """Profile, joints, frames, gripper and poses all come back."""
    semantics = RobotClient._parse_semantics(ET.fromstring(DESCRIPTION))
    assert semantics['profile'] == 'five_v1'
    assert semantics['arm_joints'] == ['a', 'b', 'c']
    assert (semantics['base_frame'], semantics['tool_frame']) == ('base', 'tip')
    assert semantics['gripper'] == {'joint': 'jaw', 'open': 1.4, 'closed': 0.0}
    assert semantics['poses'] == {'home': [0.0, 0.0, 0.0],
                                  'ready': [0.1, 0.2, 0.3]}


def test_a_description_without_semantics_is_an_empty_answer_not_an_error():
    """
    A plain URDF parses to {}.

    Such a robot can still be moved joint by joint. It has no named
    poses, and asking for one says so -- see the next test.
    """
    assert RobotClient._parse_semantics(ET.fromstring('<robot name="x"/>')) == {}


def _client(semantics, driven):
    """Build just enough of a RobotClient to call the pose methods on."""
    fake = types.SimpleNamespace(
        semantics=lambda: semantics, arm_joints=lambda: driven)
    fake.named_poses = lambda: RobotClient.named_poses(fake)
    return fake


def test_asking_a_robot_with_no_poses_for_one_explains_itself():
    """No poses is a message about the profile, not a KeyError."""
    with pytest.raises(ValueError, match='declares no named poses'):
        RobotClient.pose(_client({}, ['a']), 'home')


def test_an_unknown_pose_lists_the_ones_that_exist():
    """A typo gets the menu."""
    semantics = RobotClient._parse_semantics(ET.fromstring(DESCRIPTION))
    with pytest.raises(ValueError, match="one of \\['home', 'ready'\\]"):
        RobotClient.pose(_client(semantics, ['a', 'b', 'c']), 'redy')


def test_poses_are_mapped_to_the_controllers_joint_order_by_name():
    """
    A pose follows the joints it names, not the positions in a list.

    The profile orders its poses by its own `arm.joints`; the controller
    drives in the order of its own `joints` parameter. If those two
    files ever list the same joints differently, sending the list as
    written would put the shoulder's angle on the elbow.
    """
    semantics = RobotClient._parse_semantics(ET.fromstring(DESCRIPTION))
    poses = RobotClient.named_poses(_client(semantics, ['c', 'a', 'b']))
    assert poses['ready'] == [0.3, 0.1, 0.2]


def test_poses_for_joints_the_controller_does_not_drive_are_refused():
    """
    A four-joint pose block on a five-joint controller names both lists.

    This is the original failure -- "expected 5 joint positions, got
    4" -- with the cause in the message this time.
    """
    semantics = RobotClient._parse_semantics(ET.fromstring(DESCRIPTION))
    with pytest.raises(ValueError) as excinfo:
        RobotClient.named_poses(_client(semantics, ['a', 'b', 'c', 'd']))
    assert "['a', 'b', 'c']" in str(excinfo.value)
    assert "['a', 'b', 'c', 'd']" in str(excinfo.value)


# -- the scenarios, against robots that share nothing ------------------

class StandInRobot:
    """
    A robot that goes exactly where it is told, at once.

    Implements the part of RobotClient the scenarios use, over a plain
    dictionary of joint positions. Enforces its limits the way the real
    client does, because safety_limit depends on being refused.
    """

    def __init__(self, joints, limits, poses, gripper, followers=()):
        self._joints = list(joints)
        self._limits = dict(limits)
        self._poses = {k: list(v) for k, v in poses.items()}
        self._gripper = dict(gripper)
        self._followers = list(followers)
        self.state = {j: 0.0 for j in self._joints}
        self.state[gripper['joint']] = gripper['closed']
        #: Set to make the gripper ignore commands, to prove a scenario
        #: can fail here.
        self.gripper_stuck = False

    def arm_joints(self):
        return list(self._joints)

    def get_limits(self, joint):
        return self._limits.get(joint)

    def get_joint_state(self):
        return dict(self.state)

    def resolve(self, name):
        return name

    def pose(self, name):
        return list(self._poses[name])

    def at_pose(self, target, tolerance=0.02):
        return all(abs(self.state[j] - t) <= tolerance
                   for j, t in zip(self._joints, target))

    def move_joints(self, positions, duration=2.0):
        assert len(positions) == len(self._joints)
        for joint, position in zip(self._joints, positions):
            lower, upper = self._limits[joint]
            if not lower <= position <= upper:
                raise LimitViolation(f'{joint}: {position}')
        self.state.update(zip(self._joints, positions))

    def move_joint(self, name, position, duration=2.0):
        target = [self.state[j] for j in self._joints]
        target[self._joints.index(name)] = float(position)
        self.move_joints(target, duration)

    def move_to_named(self, name, duration=2.0):
        self.move_joints(self.pose(name), duration)

    def home(self, duration=2.5):
        self.move_to_named('home', duration)

    def gripper_joint(self):
        return self._gripper['joint']

    def gripper_travel(self):
        return self._gripper['closed'], self._gripper['open']

    def gripper_followers(self):
        return list(self._followers)

    def open_gripper(self):
        if not self.gripper_stuck:
            self.state[self._gripper['joint']] = self._gripper['open']

    def close_gripper(self, max_effort=5.0):
        if not self.gripper_stuck:
            self.state[self._gripper['joint']] = self._gripper['closed']

    def get_translation(self, parent, child, timeout=2.0):
        # Two fingers mirrored about y, as far apart as the joint is open.
        offset = 0.010 + self.state[self._gripper['joint']]
        sign = 1.0 if 'left' in child else -1.0
        return (0.0, sign * offset, 0.025)


def four_joint_prismatic():
    """Build something shaped like the 3VN arm: 4 joints, fingers in metres."""
    joints = ['shoulder_pan_joint', 'shoulder_lift_joint',
              'elbow_joint', 'wrist_joint']
    return StandInRobot(
        joints,
        limits={'shoulder_pan_joint': (-math.pi / 2, math.pi / 2),
                'shoulder_lift_joint': (-0.26, 2.09),
                'elbow_joint': (-2.09, 0.17),
                'wrist_joint': (-math.pi / 2, math.pi / 2),
                'gripper_left_finger_joint': (0.0, 0.018)},
        poses={'home': [0.0, 0.0, 0.0, 0.0], 'ready': [0.0, 0.5, -0.4, 0.0],
               'pick': [0.6, 0.9, -0.7, 0.2], 'place': [-0.6, 0.9, -0.7, 0.2]},
        gripper={'joint': 'gripper_left_finger_joint',
                 'open': 0.018, 'closed': 0.0},
        followers=['gripper_right_finger_joint'])


def five_joint_revolute():
    """Build something shaped like the SO-101: 5 joints, one jaw in radians."""
    joints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
              'wrist_flex', 'wrist_roll']
    return StandInRobot(
        joints,
        limits={'shoulder_pan': (-1.92, 1.92), 'shoulder_lift': (-1.75, 1.75),
                'elbow_flex': (-1.69, 1.69), 'wrist_flex': (-1.66, 1.66),
                'wrist_roll': (-2.74, 2.84), 'gripper': (-0.17, 1.75)},
        poses={'home': [0.0] * 5, 'ready': [0.0, -0.6, 0.8, 0.5, 0.0],
               'pick': [-0.6, -0.2, 0.5, 0.9, 0.0],
               'place': [0.6, -0.2, 0.5, 0.9, 0.0]},
        gripper={'joint': 'gripper', 'open': 1.4, 'closed': 0.0})


@pytest.fixture
def spinless(monkeypatch):
    """Neutralise rclpy.spin_once so a scenario runs without a node."""
    import sys
    fake = types.ModuleType('rclpy')
    fake.spin_once = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, 'rclpy', fake)


SUITE = ['home', 'move_joint', 'move_to_position', 'gripper',
         'safety_limit', 'pick_and_place']


@pytest.mark.parametrize('scenario', SUITE)
@pytest.mark.parametrize('robot', [four_joint_prismatic, five_joint_revolute],
                         ids=['four-joint-prismatic', 'five-joint-revolute'])
def test_every_scenario_runs_on_either_robot(spinless, robot, scenario):
    """
    The same six scenarios pass on two robots with nothing in common.

    A scenario that named `shoulder_pan_joint`, sent a four-element
    pose, or opened the gripper to 0.018 would pass on the first and
    fail on the second.
    """
    result = REGISTRY[scenario](robot()).execute()
    assert result.passed, result.report()


def test_move_joint_jogs_each_joint_inside_its_own_limits(spinless):
    """
    The jog direction follows the joint's travel.

    The four-joint robot's elbow runs from -120 to +10 degrees. A fixed
    positive jog would be refused there; the scenario has to go the
    other way, and say so in the step name.
    """
    result = REGISTRY['move_joint'](four_joint_prismatic()).execute()
    names = [step.name for step in result.steps]
    assert 'move elbow_joint to -0.40' in names
    assert 'move shoulder_pan_joint to +0.39' in names
    assert 'return elbow_joint' in names


def test_the_mirror_check_only_runs_where_there_is_a_mimic(spinless):
    """
    A single-jaw gripper gets no "both fingers" step.

    Left out, not passed: a report must never claim a check that did
    not happen.
    """
    def steps(robot):
        return [s.name for s in REGISTRY['gripper'](robot).execute().steps]

    assert 'both fingers moved together' in steps(four_joint_prismatic())
    assert 'both fingers moved together' not in steps(five_joint_revolute())


def test_a_gripper_that_does_not_move_fails_the_scenario(spinless, monkeypatch):
    """
    The stand-in can fail.

    Without this, six green results above could mean the scenarios
    assert nothing against a robot that does whatever it is told.
    """
    import time
    robot = five_joint_revolute()
    robot.gripper_stuck = True
    # The step polls until a wall-clock deadline. Make that deadline
    # arrive at once rather than waiting fifteen seconds for a failure.
    clock = iter(range(0, 100000, 100))
    monkeypatch.setattr(time, 'monotonic', lambda: float(next(clock)))
    result = REGISTRY['gripper'](robot).execute()
    assert not result.passed
    assert result.steps[-1].name == 'open gripper'

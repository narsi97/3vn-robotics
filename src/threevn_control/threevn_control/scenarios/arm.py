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
The 3VN scenario suite.

Every scenario here is written against RobotClient alone and therefore
runs unchanged against mock, Gazebo, and later the physical arm. That is
the point: a scenario that passes in simulation is a statement about the
robot, not about the simulator.

They also name no joint, no pose value and no gripper width. Which
joints the arm has comes from its controller; its named poses, its
limits and how far its gripper opens come from the description it is
running. So the same six scenarios run against the 3VN arm and against
an arm with a different number of joints, different names and a
revolute jaw, and a failure on either is a statement about that robot.
"""

from threevn_control.scenarios.base import register, Scenario

#: How far move_joint and safety_limit ask a joint to go, in radians.
JOG = 0.4

#: Fraction of the gripper's travel it may miss either end by. A share
#: of the travel, not a length: a gripper may be a prismatic finger
#: measured in metres or a revolute jaw measured in radians, and 4 mm
#: means nothing to the second.
GRIPPER_TOLERANCE = 0.22


def gripper_is_open(robot):
    """Return True when the gripper joint is at the open end of its travel."""
    closed, opened = robot.gripper_travel()
    position = robot.get_joint_state()[robot.gripper_joint()]
    return abs(position - opened) <= GRIPPER_TOLERANCE * abs(opened - closed)


def gripper_is_closed(robot):
    """Return True when the gripper joint is at the closed end of its travel."""
    closed, opened = robot.gripper_travel()
    position = robot.get_joint_state()[robot.gripper_joint()]
    return abs(position - closed) <= GRIPPER_TOLERANCE * abs(opened - closed)


@register
class Home(Scenario):
    """Return to the reference configuration and confirm arrival."""

    name = 'home'
    description = 'Move to the reference configuration.'

    def run(self):
        """Move home and verify."""
        self.step(
            'move to home',
            lambda: self.robot.home(),
            lambda: self.robot.at_pose(self.robot.pose('home')),
        )


@register
class MoveJoint(Scenario):
    """Move one joint at a time and confirm the others stay put."""

    name = 'move_joint'
    description = 'Drive each joint individually; the rest must not move.'

    def run(self):
        """Jog each arm joint in turn, and put it back."""
        self.step('home first', lambda: self.robot.home())

        joints = self.robot.arm_joints()
        for joint in joints:
            start = self.robot.get_joint_state()
            target = self._target_for(joint)
            others = [j for j in joints if j != joint]

            def move(j=joint, t=target):
                self.robot.move_joint(j, t)

            def back(j=joint, s=start):
                self.robot.move_joint(j, s[j])

            def arrived(want, j=joint, rest=others, s=start):
                state = self.robot.get_joint_state()
                if abs(state[j] - want) > 0.02:
                    return False
                # The other joints were commanded to hold their previous
                # value, so drift here means the trajectory controller is
                # not actually holding them.
                return all(abs(state[o] - s[o]) <= 0.05 for o in rest)

            self.step(f'move {joint} to {target:+.2f}', move,
                      lambda t=target: arrived(t))
            # Back before the next joint. Jogs used to accumulate, which
            # was survivable on one arm with hand-picked targets and is
            # not in general: three joints each a little forward of home
            # put a longer arm's gripper through the bench. One joint
            # away from the reference configuration at a time is a
            # motion any arm can make.
            self.step(f'return {joint}', back,
                      lambda j=joint, s=start: arrived(s[j]))

    def _target_for(self, joint):
        """
        Choose somewhere legal to send `joint`, from its own limits.

        Toward whichever limit is further from zero, and no more than a
        quarter of the way there. The direction matters: the 3VN arm's
        elbow travels -120 to +10 degrees, so a fixed +0.4 rad would be
        refused, and a list of hand-picked targets is a list of facts
        about one robot.
        """
        lower, upper = self.robot.get_limits(joint) or (-JOG, JOG)
        if upper >= -lower:
            return round(min(JOG, 0.25 * upper), 3)
        return round(max(-JOG, 0.25 * lower), 3)


@register
class MoveToPosition(Scenario):
    """Move through several named poses."""

    name = 'move_to_position'
    description = 'Traverse the named joint-space poses.'

    def run(self):
        """Visit each named pose."""
        for pose in ('home', 'ready', 'pick', 'place', 'home'):
            self.step(
                f'move to {pose}',
                lambda p=pose: self.robot.move_to_named(p),
                lambda p=pose: self.robot.at_pose(self.robot.pose(p)),
            )


@register
class Gripper(Scenario):
    """Open and close the gripper, and confirm any mimic joint follows."""

    name = 'gripper'
    description = 'Open and close the gripper; every jaw must move.'

    def run(self):
        """Cycle the gripper."""
        self.step('home first', lambda: self.robot.home())

        self.step(
            'open gripper',
            lambda: self.robot.open_gripper(),
            lambda: gripper_is_open(self.robot),
            settle=1.0,
        )

        if self.robot.gripper_followers():
            self.step(
                'both fingers moved together',
                lambda: None,
                self._fingers_mirrored,
                settle=0.2,
            )

        self.step(
            'close gripper',
            lambda: self.robot.close_gripper(),
            lambda: gripper_is_closed(self.robot),
            settle=1.0,
        )

    def _fingers_mirrored(self):
        """
        Confirm the follower finger tracks the commanded one through <mimic>.

        Only run on a gripper that HAS a follower: a single moving jaw
        has nothing to mirror, and the step is left out rather than
        passed, so the report never claims a check that did not happen.

        Checked through TF, not /joint_states. The mimic joint has no
        ros2_control interface by design - only one finger is
        commanded - so it never appears in /joint_states at all. It is
        robot_state_publisher that resolves the mimic and publishes the
        transform.

        Looking in /joint_states for it reports a perfectly working
        gripper as broken, which is exactly what this check did first.
        """
        base = self.robot.resolve('gripper_base_link')
        left = self.robot.get_translation(
            base, self.robot.resolve('gripper_left_finger_link'))
        right = self.robot.get_translation(
            base, self.robot.resolve('gripper_right_finger_link'))
        # Mirrored about the gripper's y axis: same magnitude, opposite sign.
        return (abs(left[1] + right[1]) <= 0.002
                and abs(abs(left[1]) - abs(right[1])) <= 0.002
                and abs(left[1]) > 0.002)


@register
class SafetyLimit(Scenario):
    """An out-of-limit command must be rejected, and the joint must not move."""

    name = 'safety_limit'
    description = 'An out-of-limit command is rejected and the joint stays put.'

    #: How far past the limit to ask for, in radians. A quarter turn:
    #: far enough that no tolerance could mistake it for a near miss.
    OVERSHOOT = 1.5708

    def run(self):
        """Command far past a joint limit and verify nothing bad happens."""
        self.step('home first', lambda: self.robot.home())

        # The first arm joint, whatever this robot calls it, and its own
        # upper limit -- 90 degrees on the 3VN arm's pan, 110 on the
        # SO-101's.
        joint = self.robot.arm_joints()[0]
        limits = self.robot.get_limits(joint)
        if limits is None:
            raise AssertionError(
                f'{joint} has no limit in the running description, so '
                'there is nothing for a limit check to enforce')
        upper = limits[1]
        before = self.robot.get_joint_state()[joint]

        self.step(
            'out-of-limit command is rejected',
            lambda: self._expect_rejection(joint, upper + self.OVERSHOOT),
            settle=0.5,
        )

        self.step(
            'joint did not move',
            lambda: None,
            lambda: abs(
                self.robot.get_joint_state()[joint] - before) <= 0.02,
            settle=0.5,
        )

        self.step(
            'joint is still inside its limit',
            lambda: None,
            lambda: abs(
                self.robot.get_joint_state()[joint]) <= upper + 0.02,
            settle=0.2,
        )

    def _expect_rejection(self, joint, position):
        """
        Assert the client refuses the command outright.

        This is validated ABOVE the hardware seam on purpose, because
        measurement showed nothing below it enforces limits: mock_components
        drives shoulder_pan to a full 180 degrees despite both the URDF
        <limit> and the command_interface min/max saying 90. Gazebo only
        appears to respect the limit because its physics joint has a hard
        stop - a property of the simulator, not of our software.

        A check that only passed under Gazebo would therefore be testing
        the simulator. This one tests the robot software, so it holds for
        mock, Gazebo and the ESP32 alike.
        """
        from threevn_control.robot_client import LimitViolation
        try:
            self.robot.move_joint(joint, position, duration=2.0)
        except LimitViolation:
            return True
        raise AssertionError(
            f'commanding {joint} to {position:.4f} rad was NOT '
            'rejected - the limit check is not working')


@register
class PickAndPlace(Scenario):
    """The end-to-end sequence: approach, grasp, move, release."""

    name = 'pick_and_place'
    description = 'home -> pick -> grasp -> place -> release -> home.'

    def run(self):
        """Run the full pick-and-place sequence."""
        self.step(
            'home',
            lambda: self.robot.home(),
            lambda: self.robot.at_pose(self.robot.pose('home')))

        self.step(
            'open gripper',
            lambda: self.robot.open_gripper(),
            lambda: gripper_is_open(self.robot))

        self.step(
            'move to pick',
            lambda: self.robot.move_to_named('pick', duration=2.5),
            lambda: self.robot.at_pose(self.robot.pose('pick')))

        # No post-condition, on purpose: with an object between the jaws
        # the gripper is SUPPOSED to stop short of closed, and this
        # scenario does not know whether there is one.
        self.step(
            'close gripper',
            lambda: self.robot.close_gripper(),
            settle=1.0)

        self.step(
            'move to place',
            lambda: self.robot.move_to_named('place', duration=3.0),
            lambda: self.robot.at_pose(self.robot.pose('place')))

        # Opening has nothing to stop it, so it is checked -- and this
        # check is the one that found something. These two steps used to
        # be bare waits. On a foreign arm in the simulator, closing with
        # the gripper pointing down ran the jaw into its hard stop, where
        # it stayed for good; the release "succeeded" by stalling, and
        # the scenario passed on a gripper that would never open again.
        self.step(
            'open gripper',
            lambda: self.robot.open_gripper(),
            lambda: gripper_is_open(self.robot))

        self.step(
            'return home',
            lambda: self.robot.home(),
            lambda: self.robot.at_pose(self.robot.pose('home')))

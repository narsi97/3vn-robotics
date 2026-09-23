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
"""

from threevn_control.robot_client import ARM_JOINTS, POSES
from threevn_control.scenarios.base import register, Scenario


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
            lambda: self.robot.at_pose(POSES['home']),
        )


@register
class MoveJoint(Scenario):
    """Move one joint at a time and confirm the others stay put."""

    name = 'move_joint'
    description = 'Drive each joint individually; the rest must not move.'

    def run(self):
        """Move each arm joint in turn."""
        self.step('home first', lambda: self.robot.home())

        for joint, target in zip(ARM_JOINTS, (0.4, 0.5, -0.4, 0.3)):
            others = [j for j in ARM_JOINTS if j != joint]

            def move(j=joint, t=target):
                self.robot.move_joint(j, t)

            def check(j=joint, t=target, rest=others):
                state = self.robot.get_joint_state()
                if abs(state[j] - t) > 0.02:
                    return False
                # The other joints were commanded to hold their previous
                # value, so drift here means the trajectory controller is
                # not actually holding them.
                return all(abs(state[o] - self._expected[o]) <= 0.05
                           for o in rest)

            self._expected = self.robot.get_joint_state()
            self.step(f'move {joint} to {target:+.2f}', move, check)
            self._expected[joint] = target


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
                lambda p=pose: self.robot.at_pose(POSES[p]),
            )


@register
class Gripper(Scenario):
    """Open and close the gripper, and confirm the mimic joint follows."""

    name = 'gripper'
    description = 'Open and close the gripper; both fingers must move.'

    OPEN = 0.018

    def run(self):
        """Cycle the gripper."""
        self.step('home first', lambda: self.robot.home())

        self.step(
            'open gripper',
            lambda: self.robot.open_gripper(),
            lambda: abs(self.robot.get_joint_state()['gripper_left_finger_joint']
                        - self.OPEN) <= 0.004,
            settle=1.0,
        )

        self.step(
            'both fingers moved together',
            lambda: None,
            self._fingers_mirrored,
            settle=0.2,
        )

        self.step(
            'close gripper',
            lambda: self.robot.close_gripper(),
            lambda: self.robot.get_joint_state()['gripper_left_finger_joint'] <= 0.004,
            settle=1.0,
        )

    def _fingers_mirrored(self):
        """
        Confirm the right finger follows the left through the URDF mimic tag.

        Checked through TF, not /joint_states. The mimic joint has no
        ros2_control interface by design - only the left finger is
        commanded - so it never appears in /joint_states at all. It is
        robot_state_publisher that resolves the mimic and publishes the
        transform.

        Looking in /joint_states for it reports a perfectly working
        gripper as broken, which is exactly what this check did first.
        """
        left = self.robot.get_translation(
            'gripper_base_link', 'gripper_left_finger_link')
        right = self.robot.get_translation(
            'gripper_base_link', 'gripper_right_finger_link')
        # Mirrored about the gripper's y axis: same magnitude, opposite sign.
        return (abs(left[1] + right[1]) <= 0.002
                and abs(abs(left[1]) - abs(right[1])) <= 0.002
                and abs(left[1]) > 0.002)


@register
class SafetyLimit(Scenario):
    """An out-of-limit command must be rejected, and the joint must not move."""

    name = 'safety_limit'
    description = 'An out-of-limit command is rejected and the joint stays put.'

    #: shoulder_pan is limited to +/- 90 deg; ask for 180.
    OVERSHOOT = 3.14159
    LIMIT = 1.5708

    def run(self):
        """Command far past a joint limit and verify nothing bad happens."""
        self.step('home first', lambda: self.robot.home())

        before = self.robot.get_joint_state()['shoulder_pan_joint']

        self.step(
            'out-of-limit command is rejected',
            self._expect_rejection,
            settle=0.5,
        )

        self.step(
            'joint did not move',
            lambda: None,
            lambda: abs(
                self.robot.get_joint_state()['shoulder_pan_joint'] - before) <= 0.02,
            settle=0.5,
        )

        self.step(
            'joint is still inside its limit',
            lambda: None,
            lambda: abs(
                self.robot.get_joint_state()['shoulder_pan_joint']) <= self.LIMIT + 0.02,
            settle=0.2,
        )

    def _expect_rejection(self):
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
            self.robot.move_joint('shoulder_pan_joint', self.OVERSHOOT, duration=2.0)
        except LimitViolation:
            return True
        raise AssertionError(
            f'commanding shoulder_pan_joint to {self.OVERSHOOT} rad was NOT '
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
            lambda: self.robot.at_pose(POSES['home']))

        self.step(
            'open gripper',
            lambda: self.robot.open_gripper(),
            settle=0.8)

        self.step(
            'move to pick',
            lambda: self.robot.move_to_named('pick', duration=2.5),
            lambda: self.robot.at_pose(POSES['pick']))

        self.step(
            'close gripper',
            lambda: self.robot.close_gripper(),
            settle=1.0)

        self.step(
            'move to place',
            lambda: self.robot.move_to_named('place', duration=3.0),
            lambda: self.robot.at_pose(POSES['place']))

        self.step(
            'open gripper',
            lambda: self.robot.open_gripper(),
            settle=0.8)

        self.step(
            'return home',
            lambda: self.robot.home(),
            lambda: self.robot.at_pose(POSES['home']))

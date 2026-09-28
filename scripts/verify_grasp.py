#!/usr/bin/env python3
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
Grasp something, and prove it.

    make play WORLD=grasp_bench &
    make grasp

Two independent questions, asked separately on purpose:

  holding()   what the ROBOT believes, from its own joint state. This
              is the check that will still exist on hardware.
  lifted      what actually happened, from the simulator's ground
              truth. This is the check that cannot be fooled.

Reporting only the first would have passed a grasp that merely shoved
the cube 5 mm sideways. Reporting only the second would prove nothing
about the robot's ability to tell, which is what it will have to rely
on when there is no simulator to ask.

There is also a CONTROL: a grasp attempted at empty air. A detector
that reports success there is not a detector, and without the control
a green result means nothing.
"""

import argparse
import math
import subprocess
import sys

import rclpy
from threevn_control.robot_client import RobotClient
from threevn_control.kinematics import Unreachable

MODEL = 'graspable_cube'
HOME = (0.200, 0.0, 0.1575)
SET_POSE = '/world/{world}/set_pose'
POSE_INFO = '/world/{world}/dynamic_pose/info'

#: A grasp counts as lifted when the object rises by more than this.
#: Larger than the settling jitter, smaller than the commanded lift.
LIFT_THRESHOLD_M = 0.02


def place(world, position):
    """Put the cube back where it started. True if the simulator agreed."""
    request = (f'name: "{MODEL}", position: {{x: {position[0]}, '
               f'y: {position[1]}, z: {position[2]}}}')
    try:
        out = subprocess.run(
            ['gz', 'service', '-s', SET_POSE.format(world=world),
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', request],
            capture_output=True, text=True, timeout=10).stdout
    except (subprocess.SubprocessError, OSError):
        return False
    return 'true' in out.lower()


def cube_pose(world):
    """Return the cube's ground-truth (x, y, z), or None."""
    from threevn_data.collect import _read_pose
    return _read_pose(POSE_INFO.format(world=world), MODEL)


def in_arm_frame(robot, world_xyz):
    """Express a world point in the arm's own base frame."""
    base = robot.get_translation('odom', robot.resolve('base_link'))
    return tuple(world_xyz[i] - base[i] for i in range(3))


def attempt(robot, world, target_world, label):
    """Try one grasp and report both verdicts."""
    before = cube_pose(world)
    target = in_arm_frame(robot, target_world)
    radius = math.hypot(target[0], target[1])
    print(f'\n  {label}')
    print(f'    target {radius * 1000:.0f} mm out, '
          f'{target[2] * 1000:+.0f} mm relative to the arm base')
    try:
        held = robot.grasp_at(target)
    except Unreachable as exc:
        print(f'    REFUSED: {exc}')
        return None
    finger = robot.get_joint_state()[
        robot.resolve('gripper_left_finger_joint')]
    print(f'    fingers stopped at {finger * 1000:5.2f} mm  -> holding={held}')

    try:
        robot.move_tool_to((target[0], target[1], target[2] + 0.09),
                           duration=2.5)
        robot._spin(2.0)
    except Unreachable:
        print('    (no lift: nowhere above it to move to)')
    after = cube_pose(world)
    rose = (after[2] - before[2]) if (after and before) else 0.0
    lifted = rose > LIFT_THRESHOLD_M
    print(f'    cube rose {rose * 1000:+.0f} mm            -> lifted={lifted}')
    return {'held': held, 'lifted': lifted, 'rose': rose, 'finger': finger}


def main(argv=None):
    """Run the grasp verification."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='grasp_bench')
    args = parser.parse_args(argv)

    rclpy.init()
    robot = RobotClient()
    robot.connect()
    failures = []

    try:
        place(args.world, HOME)
        robot._spin(1.5)
        real = attempt(robot, args.world, HOME, 'GRASP the cube')
        if real is None or not real['held'] or not real['lifted']:
            failures.append('the cube was not picked up')
        if real and real['held'] != real['lifted']:
            failures.append(
                f"holding() said {real['held']} but the cube "
                f"{'moved' if real['lifted'] else 'did not move'}")

        # Control: the same motion where there is nothing to grasp.
        robot.home()
        empty = (HOME[0], HOME[1] + 0.12, HOME[2])
        control = attempt(robot, args.world, empty, 'CONTROL: grasp empty air')
        if control is not None and control['held']:
            failures.append('holding() reported a grasp of empty air')
    finally:
        robot.destroy_node()
        rclpy.shutdown()

    print()
    if failures:
        for line in failures:
            print(f'  FAIL  {line}')
        return 1
    print('  ok    grasped, lifted, and the control did not false-positive')
    return 0


if __name__ == '__main__':
    sys.exit(main())

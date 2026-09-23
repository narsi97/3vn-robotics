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
Drive and manipulate at the same time.

    make mm-sim      # in one shell
    make mm-verify   # in another

The point of Phase 11 is not that an arm and a base exist in one URDF.
It is that the SAME commands that drove each alone still work, together,
against one controller_manager -- so nothing a client already knew about
/base_controller/cmd_vel or /arm_controller/follow_joint_trajectory had
to change.

Running them concurrently is the part worth checking. Sequentially, two
controllers that quietly share a resource still look fine.
"""

import math
import sys
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

PREFIX = 'arm_'
ARM_JOINTS = [PREFIX + n for n in ('shoulder_pan_joint', 'shoulder_lift_joint',
                                   'elbow_joint', 'wrist_joint')]
WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint']


class Rig(Node):
    """Talk to both subsystems of the composed robot."""

    def __init__(self):
        super().__init__('verify_mm')
        self.cmd = self.create_publisher(
            TwistStamped, '/base_controller/cmd_vel', 10)
        self.odom = None
        self.states = None
        self.create_subscription(
            Odometry, '/base_controller/odom', self._on_odom, 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_states, 10)
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

    def _on_odom(self, msg):
        self.odom = msg

    def _on_states(self, msg):
        self.states = dict(zip(msg.name, msg.position))

    def wait(self, seconds):
        """Spin for a wall-clock duration."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_ready(self, timeout=40.0):
        """Block until both subsystems are publishing."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None and self.states is not None:
                return True
        return False

    def drive(self, linear, angular, seconds):
        """Hold a velocity command, then stop."""
        msg = TwistStamped()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        end = time.time() + seconds
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd.publish(msg)
            time.sleep(0.02)
        stop = TwistStamped()
        for _ in range(10):
            stop.header.stamp = self.get_clock().now().to_msg()
            self.cmd.publish(stop)
            time.sleep(0.02)

    def send_arm(self, positions, seconds=3):
        """Send one trajectory point and return the goal handle future."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = seconds
        goal.trajectory.points = [point]
        return self.arm.send_goal_async(goal)


def pose(rig):
    """Return (x, y) from odometry."""
    p = rig.odom.pose.pose.position
    return p.x, p.y


def main():
    """Run the checks and report."""
    rclpy.init()
    rig = Rig()

    if not rig.wait_until_ready():
        print('  the composed robot is not publishing -- is `make mm-sim` up?')
        rig.destroy_node()
        rclpy.shutdown()
        return 1

    print()
    missing = [j for j in ARM_JOINTS + WHEELS if j not in rig.states]
    if missing:
        print(f'  FAIL  /joint_states is missing {missing}')
        rig.destroy_node()
        rclpy.shutdown()
        return 1
    print(f'  ok    one /joint_states carries all {len(ARM_JOINTS)} arm '
          f'joints and all {len(WHEELS)} wheels')

    if not rig.arm.wait_for_server(timeout_sec=20.0):
        print('  FAIL  the arm action server never appeared')
        rig.destroy_node()
        rclpy.shutdown()
        return 1
    print('  ok    /arm_controller/follow_joint_trajectory is available')

    # Drive and manipulate CONCURRENTLY. The base command runs on a
    # thread while the arm trajectory is in flight; a resource conflict
    # between the two hardware components shows up here and nowhere else.
    start = pose(rig)
    arm_start = rig.states[ARM_JOINTS[0]]

    future = rig.send_arm([0.4, 0.3, -0.2, 0.1])
    driver = threading.Thread(target=rig.drive, args=(0.25, 0.0, 3.0))
    driver.start()
    rig.wait(5.0)
    driver.join()
    rig.wait(1.0)

    moved = math.dist(start, pose(rig))
    turned = abs(rig.states[ARM_JOINTS[0]] - arm_start)

    accepted = future.done() and future.result() is not None \
        and future.result().accepted
    print(f'  {"ok  " if accepted else "FAIL"}  the arm goal was '
          f'{"accepted" if accepted else "REJECTED"} while the base was driving')
    print(f'  {"ok  " if moved > 0.2 else "FAIL"}  the base travelled '
          f'{moved:.3f} m while the arm was moving')
    print(f'  {"ok  " if turned > 0.2 else "FAIL"}  '
          f'{ARM_JOINTS[0]} moved {turned:.3f} rad while the base was driving')

    print()
    print('  Same topics, same action, same controller_manager as each')
    print('  subsystem alone. Composition did not change the interface.')
    print()

    ok = accepted and moved > 0.2 and turned > 0.2
    rig.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

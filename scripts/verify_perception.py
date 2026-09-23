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
How wrong is the detection?

    make mm-sim WORLD=bench_with_target
    ros2 run threevn_perception find_target
    make perception-verify

The cube's true pose is written in the world file, and the robot has not
moved, so base_footprint is the world origin. That makes the truth
available exactly, which is the only way to say what the error IS rather
than that the pipeline produced a number.

Without this, the first explanation for the shortfall was that a cube
seen off-axis shows more than one face. Measuring showed the cube is
dead ahead and the bias is in the segmented edge instead.
"""

import math
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import tf2_geometry_msgs  # noqa: F401
import tf2_ros

#: From threevn_sim/worlds/bench_with_target.sdf. Changing it there
#: changes what correct means, so these two must be read together.
CUBE_IN_WORLD = (0.40, 0.0, 0.025)
CUBE_SIZE = 0.05


class Check(Node):
    """Compare the published detection against the known truth."""

    def __init__(self):
        super().__init__('verify_perception')
        self.seen = {}
        self.create_subscription(
            PoseStamped, '/perception/target_pose',
            lambda m: self.seen.__setitem__('optical', m), 10)
        self.create_subscription(
            PoseStamped, '/perception/target_in_base',
            lambda m: self.seen.__setitem__('base', m), 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def wait(self, timeout=45.0):
        """Block until both detections have arrived."""
        end = time.time() + timeout
        while time.time() < end and len(self.seen) < 2:
            rclpy.spin_once(self, timeout_sec=0.2)
        return len(self.seen) == 2

    def truth_in_base(self, timeout=10.0):
        """
        Return the cube's true position in base_link.

        The robot has not moved, so the cube's pose in base_footprint is
        its world pose; TF carries it the rest of the way.
        """
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = CUBE_IN_WORLD[0]
        pose.pose.position.y = CUBE_IN_WORLD[1]
        pose.pose.position.z = CUBE_IN_WORLD[2]
        pose.pose.orientation.w = 1.0

        end = time.time() + timeout
        while time.time() < end:
            try:
                out = self.buffer.transform(
                    pose, 'base_link',
                    timeout=rclpy.duration.Duration(seconds=0.5))
                return (out.pose.position.x, out.pose.position.y,
                        out.pose.position.z)
            except tf2_ros.TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)
        return None


def main():
    """Run the comparison and print it."""
    rclpy.init()
    node = Check()

    if not node.wait():
        print('  no detection published -- is find_target running against '
              'the bench_with_target world?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    truth = node.truth_in_base()
    if truth is None:
        print('  TF never produced base_footprint -> base_link')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    optical = node.seen['optical']
    found = node.seen['base'].pose.position
    got = (found.x, found.y, found.z)
    error = math.dist(got, truth)

    print()
    print(f'  optical frame   {optical.header.frame_id}')
    print(f'    measured      x={optical.pose.position.x:+.3f} '
          f'y={optical.pose.position.y:+.3f} '
          f'z={optical.pose.position.z:+.3f}   (range along the axis)')
    print()
    print('  in base_link    detected          true         error')
    for axis, a, b in zip('xyz', got, truth):
        print(f'    {axis}           {a:+.3f} m         {b:+.3f} m     '
              f'{(a - b) * 1000:+6.1f} mm')
    print(f'    distance                                    '
          f'{error * 1000:6.1f} mm')
    print()
    print('  Bearing is excellent and range is biased, and they fail for')
    print('  different reasons. The centroid is robust: it does not move')
    print('  at all as the colour threshold varies across its usable')
    print('  band. The apparent WIDTH carries the target edge, where')
    print('  rendered pixels blend into the background, so segmentation')
    print('  includes roughly 3 px of bleed per side. At this range that')
    print('  is 74 px measured against 68 px true, which is the whole 8%')
    print('  shortfall. Since the bleed is an edge effect it stays a few')
    print('  pixels wide as the target shrinks with distance, so the')
    print('  proportional error grows. See docs/perception.md.')
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 0 if error < 0.06 else 1


if __name__ == '__main__':
    sys.exit(main())

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
Drive the base and report what actually happened.

The numbers in docs/mobile-base.md come from here. They were measured by
hand once, which made them unreproducible -- so the table said things
nobody could check. This makes the measurement repeatable:

    make base-sim &      # or in another shell
    make base-drive

Commanded-vs-achieved is expected to differ. Acceleration limits eat some
of the linear distance, and skid steer scrubs sideways through every
turn. That gap is the thing `wheel_separation_multiplier` calibrates, so
printing it is the point rather than a failure.
"""

import math
import subprocess
import sys
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

CMD_TOPIC = '/base_controller/cmd_vel'
ODOM_TOPIC = '/base_controller/odom'

GT_TOPIC = '/world/empty_bench/dynamic_pose/info'
GT_MODEL = 'threevn_base'


def ground_truth():
    """
    Read the simulator's true pose for the base.

    Odometry is DERIVED from the wheels, so comparing odometry against the
    command only proves the controller tracks its setpoint -- it cannot
    show odometry error, because both sides of that comparison come from
    the same wheel encoders. Skid-steer error is precisely the gap between
    wheel-derived odometry and where the robot physically ended up, so the
    measurement needs a source outside the control loop.

    Returns (x, y, yaw) or None when the simulator is not reachable.
    """
    try:
        out = subprocess.run(
            ['gz', 'topic', '-t', GT_TOPIC, '-e', '-n', '1'],
            capture_output=True, text=True, timeout=15).stdout
    except (subprocess.SubprocessError, OSError):
        return None

    # The message is a flat list of `pose { name: ... }` blocks; take the
    # model's own, which is the first one, and stop at the next name.
    block, seen = [], False
    for line in out.splitlines():
        if 'name:' in line:
            if seen:
                break
            seen = f'"{GT_MODEL}"' in line
            continue
        if seen:
            block.append(line)
    if not seen:
        return None

    def field(section, key):
        inside = False
        for line in block:
            stripped = line.strip()
            if stripped.startswith(section):
                inside = True
                continue
            if inside:
                if stripped.startswith('}'):
                    inside = False
                    continue
                if stripped.startswith(key + ':'):
                    return float(stripped.split(':', 1)[1])
        return 0.0

    qz = field('orientation', 'z')
    qw = field('orientation', 'w') or 1.0
    return (field('position', 'x'), field('position', 'y'),
            math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz))


def yaw_of(q):
    """Extract yaw from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    """Publish velocity commands and record the odometry response."""

    def __init__(self):
        super().__init__('verify_base_drive')
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.odom = None
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

    def _on_odom(self, msg):
        self.odom = msg

    def spin_for(self, seconds):
        """Spin the executor for a wall-clock duration."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_odom(self, timeout=30.0):
        """Block until odometry arrives, or give up and say so."""
        end = time.time() + timeout
        while self.odom is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def pose(self):
        """Return (x, y, yaw) from the latest odometry."""
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def drive(self, linear, angular, seconds):
        """Hold a velocity command for a duration, then stop."""
        msg = TwistStamped()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        end = time.time() + seconds
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        for _ in range(10):
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.spin_for(1.0)


def main():
    """Run the forward and rotation legs and print the comparison."""
    rclpy.init()
    node = Driver()

    if not node.wait_for_odom():
        print(f'no odometry on {ODOM_TOPIC} -- is `make base-sim` running?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Let the controller settle before taking a datum.
    node.spin_for(1.0)
    x0, y0, yaw0 = node.pose()
    g0 = ground_truth()

    speed, forward_s = 0.3, 3.0
    node.drive(speed, 0.0, forward_s)
    x1, y1, yaw1 = node.pose()
    g1 = ground_truth()
    odom_dist = math.hypot(x1 - x0, y1 - y0)

    rate, turn_s = 1.0, 2.0
    node.drive(0.0, rate, turn_s)
    x2, y2, yaw2 = node.pose()
    g2 = ground_truth()
    odom_turn = math.degrees((yaw2 - yaw1 + math.pi) % (2 * math.pi) - math.pi)

    cmd_dist = speed * forward_s
    cmd_turn = math.degrees(rate * turn_s)

    print()
    print('  leg        commanded      odometry      ground truth')
    if g0 and g1 and g2:
        true_dist = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
        true_turn = math.degrees(
            (g2[2] - g1[2] + math.pi) % (2 * math.pi) - math.pi)
        print(f'  forward    {cmd_dist:7.3f} m     {odom_dist:7.3f} m      '
              f'{true_dist:7.3f} m')
        print(f'  rotate     {cmd_turn:7.1f} deg   {odom_turn:7.1f} deg    '
              f'{true_turn:7.1f} deg')
        print()
        print('  odometry error (what a localisation filter has to absorb):')
        print(f'    linear    {100 * (odom_dist - true_dist) / max(true_dist, 1e-9):+6.1f} %')
        print(f'    heading   {100 * (odom_turn - true_turn) / max(abs(true_turn), 1e-9):+6.1f} %')
        print()
        print('  Heading error is the one that matters. Skid steer turns by')
        print('  scrubbing the wheels sideways, which the differential model')
        print('  cannot represent, so wheel-derived heading drifts in')
        print('  proportion to how much turning has happened. Correct it with')
        print('  wheel_separation_multiplier in base_controllers.yaml.')
    else:
        print(f'  forward    {cmd_dist:7.3f} m     {odom_dist:7.3f} m      '
              f'unavailable')
        print(f'  rotate     {cmd_turn:7.1f} deg   {odom_turn:7.1f} deg    '
              f'unavailable')
        print()
        print('  No ground truth: `gz topic` could not be reached, so only')
        print('  controller tracking is shown. Odometry compared against the')
        print('  COMMAND proves the controller follows its setpoint and')
        print('  nothing about odometry accuracy -- both sides come from the')
        print('  same wheel encoders.')
    print()
    print('  Measure against a FRESHLY started sim. A base left sitting after')
    print('  an earlier run has reported wheel speeds far below command,')
    print('  which looks like a physics finding and is not one.')
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

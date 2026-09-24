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
Which source of heading should the robot believe?

    make mm-sim WORLD=bench_with_target
    make heading

Turns the base through a series of rotations and compares three
headings against Gazebo ground truth:

    wheel odometry   what diff_drive_controller derives from encoders
    gyro integral    the IMU's z rate, integrated
    ground truth     the simulator's own answer

Phase 10 measured that wheel heading over-reports by 63%, and that
wheel_separation_multiplier saturates around 26% error because
skid-steer scrub is not a fixed track-width error. This is the
experiment that says whether a $2 gyroscope settles it.
"""

import math
import subprocess
import sys
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

GT_TOPIC = '/world/{world}/dynamic_pose/info'
MODEL = 'threevn_mm'


def yaw_of(q):
    """Extract yaw from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ground_truth_yaw(world):
    """Read the simulator's true yaw for the robot, or None."""
    try:
        out = subprocess.run(
            ['gz', 'topic', '-t', GT_TOPIC.format(world=world),
             '-e', '-n', '1'],
            capture_output=True, text=True, timeout=15).stdout
    except (subprocess.SubprocessError, OSError):
        return None

    block, seen = [], False
    for line in out.splitlines():
        if 'name:' in line:
            if seen:
                break
            seen = f'"{MODEL}"' in line
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

    qz, qw = field('orientation', 'z'), field('orientation', 'w') or 1.0
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


class Heading(Node):
    """Collect the three headings while driving."""

    def __init__(self, world):
        super().__init__('verify_heading')
        self.world = world
        self.cmd = self.create_publisher(
            TwistStamped, '/base_controller/cmd_vel', 10)
        self.odom = None
        self.fused = None
        self.gyro_yaw = 0.0
        self._last_imu = None
        self.create_subscription(
            Odometry, '/base_controller/odom', self._on_odom, 10)
        # Optional: only present when `make fused-odom` is running.
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda m: setattr(self, 'fused', m), 10)
        self.create_subscription(
            Imu, '/imu', self._on_imu, QoSPresetProfiles.SENSOR_DATA.value)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_imu(self, msg):
        """
        Integrate the z gyro rate.

        Dead-reckoned on purpose: this is what the raw sensor gives, with
        no filter and no correction, so the drift below is the drift a
        gyroscope alone actually has.
        """
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_imu is not None:
            dt = stamp - self._last_imu
            if 0.0 < dt < 0.5:
                self.gyro_yaw += msg.angular_velocity.z * dt
        self._last_imu = stamp

    def spin_for(self, seconds):
        """Spin the executor for a wall-clock duration."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout=45.0):
        """Block until odometry and the IMU are both publishing."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None and self._last_imu is not None:
                return True
        return False

    def turn(self, rate, seconds):
        """Rotate in place, then stop and settle."""
        msg = TwistStamped()
        msg.twist.angular.z = rate
        end = time.time() + seconds
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        stop = TwistStamped()
        for _ in range(12):
            stop.header.stamp = self.get_clock().now().to_msg()
            self.cmd.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.spin_for(1.0)


def wrap(angle):
    """Wrap to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    """Run the comparison."""
    world = sys.argv[1] if len(sys.argv) > 1 else 'bench_with_target'
    rclpy.init()
    node = Heading(world)

    if not node.wait_ready():
        print('  odometry or /imu never arrived -- is the sim running?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.spin_for(1.0)
    truth0 = ground_truth_yaw(world)
    odom0 = yaw_of(node.odom.pose.pose.orientation)
    gyro0 = node.gyro_yaw
    if truth0 is None:
        print('  no ground truth from the simulator')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    legs = [(0.8, 2.0), (-0.8, 2.0), (0.6, 3.0), (-0.6, 3.0)]
    fused0 = (yaw_of(node.fused.pose.pose.orientation)
              if node.fused is not None else None)
    if fused0 is None:
        print('  (/odometry/filtered absent: run `make fused-odom` too to '
              'include the fused estimate)')

    print()
    print('  leg   commanded    wheel odom     gyro      fused      truth')

    cumulative_cmd = 0.0
    wheel_legs, gyro_legs, fused_legs = [], [], []
    previous = {'odom': 0.0, 'gyro': 0.0, 'fused': 0.0, 'truth': 0.0}
    for rate, seconds in legs:
        node.turn(rate, seconds)
        cumulative_cmd += rate * seconds
        truth = wrap(ground_truth_yaw(world) - truth0)
        odom = wrap(yaw_of(node.odom.pose.pose.orientation) - odom0)
        gyro = wrap(node.gyro_yaw - gyro0)
        fused = (wrap(yaw_of(node.fused.pose.pose.orientation) - fused0)
                 if fused0 is not None else float('nan'))
        print('  %+.1f rad/s x %.0fs  %+7.1f deg  %+7.1f deg  %+7.1f deg  '
              '%+7.1f deg' % (
                  rate, seconds, math.degrees(odom), math.degrees(gyro),
                  math.degrees(fused), math.degrees(truth)))

        # Error on THIS leg's rotation, not on the running total.
        turned = wrap(truth - previous['truth'])
        wheel_legs.append(wrap(odom - previous['odom']) - turned)
        gyro_legs.append(wrap(gyro - previous['gyro']) - turned)
        if fused0 is not None:
            fused_legs.append(wrap(fused - previous['fused']) - turned)
        previous = {'odom': odom, 'gyro': gyro, 'fused': fused,
                    'truth': truth}

    truth = wrap(ground_truth_yaw(world) - truth0)
    odom = wrap(yaw_of(node.odom.pose.pose.orientation) - odom0)
    gyro = wrap(node.gyro_yaw - gyro0)

    print()
    print('  PER-LEG absolute error, which is the honest measure:')
    for name, values in (('wheel odometry', wheel_legs),
                         ('gyro integral', gyro_legs),
                         ('fused odometry', fused_legs)):
        if not values or any(v != v for v in values):
            continue
        worst = max(abs(v) for v in values)
        mean = sum(abs(v) for v in values) / len(values)
        print('    %-18s mean %5.1f deg   worst %5.1f deg' % (
            name, math.degrees(mean), math.degrees(worst)))

    print()
    print('  net error after %.0f degrees of NET command:' % math.degrees(
        cumulative_cmd))
    print('    wheel odometry         %+7.1f deg' % math.degrees(
        wrap(odom - truth)))
    print('    gyro integral          %+7.1f deg' % math.degrees(
        wrap(gyro - truth)))
    if fused0 is not None:
        fused_total = wrap(yaw_of(node.fused.pose.pose.orientation) - fused0)
        print('    fused odometry         %+7.1f deg' % math.degrees(
            wrap(fused_total - truth)))

    print()
    print('  READ THE PER-LEG NUMBERS, NOT THE NET ONE.')
    print()
    print('  The net figure was the first thing measured here and it is')
    print('  misleading: wheel heading error is PROPORTIONAL to rotation,')
    print('  so a sequence of equal and opposite turns cancels it. One')
    print('  run had wheel odometry net error of +0.4 deg while every')
    print('  individual leg was 30% short - a sensor reading 38.6 deg for')
    print('  a true 55.2 looking better than the gyro that read 55.1.')
    print()
    print('  Wheel heading is wrong because skid steer turns by scrubbing,')
    print('  which the differential model cannot represent, and Phase 10')
    print('  showed the standard correction saturates. A gyroscope')
    print('  measures rotation directly. It drifts instead, which is what')
    print('  the stationary bias estimate and an absolute reference are')
    print('  for - and this robot has no absolute reference at all.')
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

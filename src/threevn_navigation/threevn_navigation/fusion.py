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
Heading from the gyroscope, distance from the wheels.

WHY EACH SOURCE IS USED FOR WHAT IT IS GOOD AT, measured rather than
assumed. Driving the base through four turns and comparing against
Gazebo ground truth:

    wheel odometry heading error   -9.1 deg
    gyro integral heading error    -0.7 deg

Wheel heading is wrong because skid steer turns by scrubbing the wheels
sideways, which the differential model cannot represent. Phase 10 showed
that `wheel_separation_multiplier` - the standard correction - SATURATES
around 26% error, so no value of it fixes this. A gyroscope measures
rotation directly and does not care how the wheels got there.

Straight-line wheel odometry, by contrast, was exact to 0.0% against
ground truth. So: heading from the gyro, translation from the wheels.

WHAT THIS IS NOT. It is not an EKF and it does not pretend the result
stops drifting. A gyro integral has no absolute reference, so its error
is a random walk that grows without bound; this robot has no
magnetometer, no scan matcher and no GPS, so nothing here can correct
it. The honest claim is a much slower drift, not a fixed one.

`robot_localization` is the standard tool and would be the right answer
with more sensors to fuse. With exactly one rotation source there is
nothing to weigh against anything, and an EKF configured over a single
input is a covariance matrix wrapped around a subtraction.
"""

import math


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class YawFuser:
    """
    Integrate gyro rate into a heading, with a stationary bias estimate.

    The bias estimate is the one piece of real filtering here and it
    earns its place: a gyroscope at rest reads a small constant offset,
    and integrating that offset is what turns a drift of degrees per
    minute into degrees per second. Measuring it while the robot is
    known to be still costs nothing and removes most of the drift.
    """

    def __init__(self, bias_samples=100, still_rate=0.02):
        self.yaw = 0.0
        self.bias = 0.0
        self.still_rate = still_rate
        self.bias_samples = bias_samples
        self._bias_total = 0.0
        self._bias_count = 0
        self._last_stamp = None

    @property
    def calibrated(self):
        """True once enough stationary samples have been seen."""
        return self._bias_count >= self.bias_samples

    def observe(self, rate, stamp, moving):
        """
        Fold in one gyro reading. Returns the current heading.

        `moving` comes from the commanded or measured wheel velocity, not
        from the gyro itself: using the gyro to decide whether the robot
        is still, in order to calibrate the gyro, is a loop that
        happily calibrates away a real slow rotation.
        """
        if not moving and abs(rate) < self.still_rate \
                and not self.calibrated:
            self._bias_total += rate
            self._bias_count += 1
            self.bias = self._bias_total / self._bias_count

        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            # A gap means dropped messages or a clock jump. Integrating
            # across it invents rotation that may not have happened, and
            # a stale dt is worse than a missing sample.
            if 0.0 < dt < 0.5:
                self.yaw = wrap(self.yaw + (rate - self.bias) * dt)
        self._last_stamp = stamp
        return self.yaw

    def reset(self, yaw=0.0):
        """Set the heading, keeping the bias estimate."""
        self.yaw = wrap(yaw)
        self._last_stamp = None


class FusedPose:
    """
    Wheel translation advanced along the gyro's heading.

    The wheels say HOW FAR, the gyro says WHICH WAY. Taking the wheel
    odometry's own x and y would import the heading error it was built
    with: a differential model that thinks it turned 45 degrees when it
    turned 68 puts the distance in the wrong direction, and the position
    error grows with every leg even though the distance was right.
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self._last_distance = None

    def advance(self, wheel_distance, yaw):
        """Fold in a new cumulative wheel distance, at the given heading."""
        if self._last_distance is not None:
            step = wheel_distance - self._last_distance
            self.x += step * math.cos(yaw)
            self.y += step * math.sin(yaw)
        self._last_distance = wheel_distance
        return self.x, self.y

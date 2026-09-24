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
Heading from a gyro, on synthetic readings with known answers.

No ROS, no simulator: rates fed in directly, so a failure is the
arithmetic rather than the world.
"""

import math

import pytest
from threevn_navigation.fusion import FusedPose, YawFuser, wrap


def feed(fuser, rate, seconds, hz=100.0, moving=True, start=0.0):
    """Feed a constant rate for a duration and return the final heading."""
    step = 1.0 / hz
    stamp = start
    yaw = fuser.yaw
    for _ in range(int(seconds * hz)):
        stamp += step
        yaw = fuser.observe(rate, stamp, moving)
    return yaw, stamp


def calibrated(bias=0.0, samples=100):
    """Return a fuser that has already seen its stationary samples."""
    fuser = YawFuser(bias_samples=samples)
    stamp = 0.0
    for _ in range(samples):
        stamp += 0.01
        fuser.observe(bias, stamp, moving=False)
    return fuser, stamp


# -- integration --------------------------------------------------------

def test_a_constant_rate_integrates_to_the_expected_angle():
    """1 rad/s for 1 s is 1 radian. The base case, so the rest means something."""
    fuser, stamp = calibrated()
    yaw, _ = feed(fuser, 1.0, 1.0, start=stamp)
    assert yaw == pytest.approx(1.0, abs=0.02)


def test_opposite_turns_cancel():
    """Rotating back returns the heading, which the wrap must not break."""
    fuser, stamp = calibrated()
    _, stamp = feed(fuser, 0.8, 2.0, start=stamp)
    yaw, _ = feed(fuser, -0.8, 2.0, start=stamp)
    assert yaw == pytest.approx(0.0, abs=0.03)


def test_heading_wraps_rather_than_growing():
    """
    Past a full turn the angle must come back round.

    An unwrapped heading eventually exceeds what a quaternion round trip
    preserves, and the robot appears to spin without stopping.
    """
    fuser, stamp = calibrated()
    yaw, _ = feed(fuser, 2.0, 4.0, start=stamp)
    assert -math.pi <= yaw <= math.pi


# -- bias ---------------------------------------------------------------

def test_a_stationary_offset_is_measured_and_removed():
    """
    THE ONE PIECE OF REAL FILTERING HERE.

    A gyroscope at rest reads a small constant offset. Integrating it is
    the difference between degrees per minute of drift and degrees per
    second.
    """
    fuser, stamp = calibrated(bias=0.01)
    assert fuser.bias == pytest.approx(0.01, abs=1e-9)

    # Now genuinely still, and reading its bias: heading must not move.
    yaw, _ = feed(fuser, 0.01, 10.0, moving=False, start=stamp)
    assert yaw == pytest.approx(0.0, abs=0.01)


def test_without_bias_removal_the_same_reading_drifts():
    """The counterfactual, so the value of the bias estimate is visible."""
    naive = YawFuser(bias_samples=100)
    naive.bias = 0.0
    naive._bias_count = 100  # pretend calibration found nothing
    yaw, _ = feed(naive, 0.01, 10.0, moving=False)
    assert abs(yaw) > 0.09, 'an uncorrected 0.01 rad/s bias must drift'


def test_bias_is_not_learned_while_moving():
    """
    A ROTATION IS NOT A BIAS.

    Sampling while the robot turns would learn the turn as an offset and
    then subtract real rotation for the rest of the run.
    """
    fuser = YawFuser(bias_samples=50)
    feed(fuser, 0.5, 2.0, moving=True)
    assert fuser.bias == pytest.approx(0.0)
    assert not fuser.calibrated


def test_movement_is_judged_by_the_wheels_not_the_gyro():
    """
    Using the gyro to decide whether to trust the gyro is a loop.

    `observe` takes `moving` from outside for exactly this reason: a slow
    genuine rotation looks like a small bias to a gyro judging itself,
    and it would calibrate the rotation away.
    """
    fuser = YawFuser(bias_samples=10)
    # Reading is small enough to look like a bias, but the wheels say
    # the robot is turning.
    for index in range(50):
        fuser.observe(0.01, index * 0.01, moving=True)
    assert not fuser.calibrated


# -- robustness ---------------------------------------------------------

def test_a_gap_in_the_stream_is_not_integrated_across():
    """
    Dropped messages or a clock jump must not invent rotation.

    Integrating a stale dt turns a two-second outage into an imagined
    turn, and nothing downstream can tell the difference.
    """
    fuser, stamp = calibrated()
    fuser.observe(1.0, stamp + 0.01, moving=True)
    before = fuser.yaw
    fuser.observe(1.0, stamp + 5.0, moving=True)
    assert fuser.yaw == pytest.approx(before), 'a 5 s gap must be skipped'


def test_time_going_backwards_is_ignored():
    """A negative dt would rotate the robot the wrong way."""
    fuser, stamp = calibrated()
    fuser.observe(1.0, stamp + 1.0, moving=True)
    before = fuser.yaw
    fuser.observe(1.0, stamp + 0.5, moving=True)
    assert fuser.yaw == pytest.approx(before)


# -- position -----------------------------------------------------------

def test_distance_is_carried_along_the_given_heading():
    """
    THE WHEELS SAY HOW FAR, THE GYRO SAYS WHICH WAY.

    One metre due east then one metre due north lands at (1, 1).
    """
    pose = FusedPose()
    pose.advance(0.0, 0.0)
    pose.advance(1.0, 0.0)
    x, y = pose.advance(2.0, math.pi / 2)
    assert x == pytest.approx(1.0, abs=1e-9)
    assert y == pytest.approx(1.0, abs=1e-9)


def test_a_wrong_heading_puts_the_right_distance_in_the_wrong_place():
    """
    Why the wheel odometry's OWN x and y are not used.

    A differential model that thinks it turned 45 degrees when it turned
    68 travels the correct distance in the wrong direction, and the
    position error compounds with every leg even though the odometer was
    right.
    """
    truth = FusedPose()
    truth.advance(0.0, 0.0)
    truth.advance(1.0, math.radians(68.0))

    believed = FusedPose()
    believed.advance(0.0, 0.0)
    believed.advance(1.0, math.radians(45.0))

    apart = math.dist((truth.x, truth.y), (believed.x, believed.y))
    assert apart > 0.35, 'a 23 degree heading error over 1 m is far off'


def test_the_first_reading_only_establishes_a_datum():
    """Nothing has been travelled yet, so nothing moves."""
    pose = FusedPose()
    assert pose.advance(5.0, 1.2) == (0.0, 0.0)


# -- helper -------------------------------------------------------------

@pytest.mark.parametrize('angle', [
    0.0, math.pi, -math.pi, 3 * math.pi, -3 * math.pi, 2 * math.pi, 7.5, -7.5,
])
def test_wrap_brings_angles_into_range(angle):
    """
    Wrapping is used on every comparison; an off-by-2pi hides errors.

    Compared as ANGLES, not as numbers. Half a turn is both +pi and -pi,
    and asserting one representative tests the convention rather than
    the behaviour - which is how this test first failed on a correct
    implementation.
    """
    wrapped = wrap(angle)
    assert -math.pi <= wrapped <= math.pi
    # Same direction: the difference must be a whole number of turns.
    turns = (angle - wrapped) / (2 * math.pi)
    assert turns == pytest.approx(round(turns), abs=1e-9)

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
Tests for the closed-form inverse kinematics.

The load-bearing test is the ROUND TRIP: solve for joint angles, then
push those angles back through forward_kinematics and require the tool
to land where it was asked to. That is worth more than any hand-computed
pose, because it cannot be satisfied by an IK that is merely
self-consistent -- forward_kinematics is independent of it, was written
first, and already agrees with a running robot_state_publisher.
"""

import math
import pathlib

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from threevn_control.kinematics import (
    arm_geometry,
    forward_kinematics,
    inverse_kinematics,
    joint_limits,
    reachable,
    tool_position,
    Unreachable,
)
import xacro

SHARE = pathlib.Path(get_package_share_directory('threevn_robot_description'))
CONFIG_DIR = SHARE / 'config'
XACRO_ENTRY = SHARE / 'urdf' / 'threevn_arm.urdf.xacro'

#: Tolerance on the round trip, in metres. This is pure floating-point
#: algebra with no iteration, so anything above a micron means a bug.
TOLERANCE_M = 1e-6

#: The pitch these tests grasp at, and NOT the obvious 180 degrees.
#: Straight down is unreachable everywhere on this arm -- see
#: test_straight_down_is_impossible_on_this_arm. 90 degrees is a
#: horizontal, side-on approach, which is what it can actually do.
GRASP_PITCH = math.radians(90)


def _expand(profile):
    doc = xacro.process_file(
        str(XACRO_ENTRY),
        mappings={'params_file': str(profile), 'target': 'mock'})
    return doc.toprettyxml(indent='  ')


@pytest.fixture(scope='module')
def urdf():
    """Return the expanded URDF for the default profile."""
    return _expand(CONFIG_DIR / 'threevn_arm_v1.yaml')


@pytest.fixture(scope='module')
def geom(urdf):
    """Return the measured arm geometry."""
    return arm_geometry(urdf)


# -- the geometry is measured, not assumed -----------------------------

def test_geometry_is_measured_from_the_urdf(geom):
    """The link lengths come out of the URDF, in metres."""
    assert geom['joints'] == [
        'shoulder_pan_joint', 'shoulder_lift_joint',
        'elbow_joint', 'wrist_joint']
    assert geom['a1'] == pytest.approx(0.110, abs=1e-9)
    assert geom['a2'] == pytest.approx(0.095, abs=1e-9)
    assert geom['a3'] == pytest.approx(0.070, abs=1e-9)
    assert geom['shoulder_z'] == pytest.approx(0.065, abs=1e-9)


def test_geometry_tracks_a_changed_urdf():
    """
    Change a link, and the IK follows.

    The point of measuring rather than hardcoding: the long-reach profile
    is a different arm, and its geometry must come out different without
    anyone editing this module.
    """
    other = arm_geometry(_expand(CONFIG_DIR / 'threevn_arm_v1_long_reach.yaml'))
    default = arm_geometry(_expand(CONFIG_DIR / 'threevn_arm_v1.yaml'))
    assert other['a1'] != default['a1'] or other['a2'] != default['a2']


# -- the round trip ----------------------------------------------------

def _round_trip(urdf, geom, position, pitch):
    solution = inverse_kinematics(urdf, position, pitch=pitch, geometry=geom)
    landed = tool_position(urdf, solution, target='grasp_frame',
                           base='base_link')
    return solution, np.array(landed)


def test_round_trip_over_the_workspace(urdf, geom):
    """
    Every reachable point solved for is a point the arm actually reaches.

    Swept rather than spot-checked: a sign error in one quadrant is
    exactly the kind of bug a single hand-picked pose misses.
    """
    checked = 0
    for x in np.linspace(-0.18, 0.18, 7):
        for y in np.linspace(-0.18, 0.18, 7):
            for z in np.linspace(0.02, 0.28, 7):
                target = (float(x), float(y), float(z))
                if math.hypot(x, y) < 1e-3:
                    continue           # on the yaw axis, yaw undefined
                if not reachable(urdf, target, pitch=GRASP_PITCH,
                                 geometry=geom):
                    continue
                _, landed = _round_trip(urdf, geom, target, GRASP_PITCH)
                assert np.allclose(landed, target, atol=TOLERANCE_M), target
                checked += 1
    assert checked > 20, f'only {checked} reachable samples; sweep too narrow'


def test_round_trip_holds_for_several_pitches(urdf, geom):
    """Pitch is a real degree of freedom, not only the top-down default."""
    # High and mid-radius: the only region this arm can enter at
    # several different approach angles. Near table height it is
    # restricted to one narrow band -- see the workspace survey.
    target = (0.19, 0.0, 0.20)
    for pitch in (math.radians(d) for d in (60, 75, 90, 105)):
        if not reachable(urdf, target, pitch=pitch, geometry=geom):
            continue
        _, landed = _round_trip(urdf, geom, target, pitch)
        assert np.allclose(landed, target, atol=TOLERANCE_M), pitch


def test_the_requested_pitch_is_what_the_tool_gets(urdf, geom):
    """
    The last link points where it was told to.

    Checked through the rotation matrix rather than by summing the joint
    angles, so it tests the robot's pose and not the arithmetic that
    produced it.
    """
    target = (0.26, 0.0, 0.03)
    for pitch in (math.radians(75), math.radians(90)):
        solution = inverse_kinematics(urdf, target, pitch=pitch, geometry=geom)
        frame = forward_kinematics(urdf, solution, target='grasp_frame',
                                   base='base_link')
        # Local +Z of the tool, expressed in the base frame.
        axis = frame[:3, 2]
        assert axis[2] == pytest.approx(math.cos(pitch), abs=1e-6)


def test_straight_down_is_impossible_on_this_arm(urdf, geom):
    """
    Pin a real limitation of the robot, not of the solver.

    A top-down pick is the obvious way to lift something off a table,
    and this arm cannot do it ANYWHERE: `elbow_joint` is limited to
    [-120, +10] degrees, so it bends only one way, and no configuration
    both reaches a point and leaves the gripper pointing at 180.

    This test exists so the limitation stays visible. If someone widens
    that limit or changes the geometry, top-down may become possible --
    and this test failing is how they will find out, instead of
    rediscovering it when a grasp strategy quietly stops making sense.
    """
    for radius in (0.10, 0.15, 0.20, 0.25):
        for height in (-0.02, 0.0, 0.05, 0.12):
            assert not reachable(urdf, (radius, 0.0, height),
                                 pitch=math.pi, geometry=geom), \
                f'top-down became reachable at r={radius}, z={height}'


def test_a_side_on_grasp_is_reachable(urdf, geom):
    """The approach the arm CAN do, so the negative test above has a pair."""
    # 260 mm out, just above the base plane: near full extension, which
    # is the ONLY place this arm meets an object sitting on a table.
    assert reachable(urdf, (0.26, 0.0, 0.03), pitch=GRASP_PITCH,
                     geometry=geom)


# -- refusing, rather than missing -------------------------------------

def test_a_point_beyond_reach_raises(urdf, geom):
    """Too far away is an error, never a nearby pose returned quietly."""
    with pytest.raises(Unreachable) as excinfo:
        inverse_kinematics(urdf, (1.5, 0.0, 0.1), geometry=geom)
    assert 'mm' in str(excinfo.value)


def test_the_refusal_says_what_the_arm_can_do(urdf, geom):
    """An error that does not state the reach makes you go and measure it."""
    with pytest.raises(Unreachable) as excinfo:
        inverse_kinematics(urdf, (1.5, 0.0, 0.1), geometry=geom)
    assert '275' in str(excinfo.value)        # 110 + 95 + 70 mm


def test_solutions_respect_the_joint_limits(urdf, geom):
    """Whatever comes back is a pose the robot is allowed to hold."""
    limits = joint_limits(urdf)
    for x in np.linspace(0.06, 0.20, 8):
        for z in np.linspace(0.02, 0.24, 8):
            target = (float(x), 0.02, float(z))
            try:
                solution = inverse_kinematics(urdf, target,
                                              pitch=GRASP_PITCH, geometry=geom)
            except Unreachable:
                continue
            for name, value in solution.items():
                lower, upper = limits[name]
                assert lower - 1e-6 <= value <= upper + 1e-6, (name, target)


def test_both_elbow_solutions_reach_the_same_point(urdf, geom):
    """Elbow up and elbow down are different poses, not different places."""
    target = (0.19, 0.03, 0.20)
    landed = []
    for elbow in ('down', 'up'):
        try:
            solution = inverse_kinematics(urdf, target, elbow=elbow,
                                          pitch=GRASP_PITCH, geometry=geom)
        except Unreachable:
            continue
        landed.append(np.array(tool_position(
            urdf, solution, target='grasp_frame', base='base_link')))
    assert landed, 'neither elbow solution was available'
    for point in landed:
        assert np.allclose(point, target, atol=TOLERANCE_M)


# -- refusing an arm it does not fit -----------------------------------

def test_a_chain_of_the_wrong_shape_is_refused(urdf):
    """
    The closed form checks the robot matches it before solving.

    Applied to an arm it does not fit, a closed form does not fail --
    it returns confident nonsense. Asking for a frame partway down the
    chain gives the wrong number of driven joints, and that must raise.
    """
    with pytest.raises(Unreachable):
        arm_geometry(urdf, target='upper_arm_link')

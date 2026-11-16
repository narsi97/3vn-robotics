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
robot_state_publisher publishes TF, and it agrees with our kinematics.

Two things at once, and the second is the interesting one.

First: the description is USABLE, not merely parseable. Every fast test
so far checks the URDF as a document; this one launches
robot_state_publisher and asks tf2 for a transform, which is what every
later phase actually depends on.

Second: our forward kinematics is checked against KDL. RSP computes these
transforms through an entirely separate implementation, so agreement
between them is real evidence. A kinematics module tested only against
its own arithmetic proves only that the arithmetic is self-consistent -
including when it is consistently wrong.

This is also the file docs/simulation.md pointed at as the template for
integration tests, and which did not previously exist.
"""

import math
import tempfile

from conftest import (
    joint_positions,
    legal_targets,
    REQUIRED_FRAMES,
    ROBOT,
    run as _run,
    send_arm_goal,
)
import numpy as np
import pytest

pytestmark = pytest.mark.ros

STARTUP_TIMEOUT = 60.0

#: Frames every later phase depends on, for the robot under test.
FRAMES = REQUIRED_FRAMES

#: The frame the others are measured from.
BASE = ROBOT['base_frame']


def _tf_translation(parent, child, timeout=8):
    """Ask tf2 for a translation, returning (x, y, z)."""
    code, out = _run(
        ['timeout', str(timeout), 'ros2', 'run', 'tf2_ros', 'tf2_echo',
         parent, child], timeout=timeout + 8)
    for line in reversed(out.splitlines()):
        if 'Translation:' in line:
            body = line.split('[', 1)[1].split(']', 1)[0]
            return tuple(float(v) for v in body.split(','))
    raise AssertionError(f'no transform {parent} -> {child}:\n{out}')


@pytest.mark.parametrize('frame', FRAMES)
def test_tf_resolves_every_required_frame(robot, frame):
    """
    tf2 can resolve each frame later phases depend on.

    A frame that parses in the URDF but never reaches TF is a frame that
    does not exist as far as perception or planning are concerned.
    """
    x, y, z = _tf_translation(BASE, frame)
    assert all(math.isfinite(v) for v in (x, y, z))


def test_tf_tree_has_a_single_root(robot):
    """
    The TF tree is one connected tree, not a forest.

    Two roots means something is publishing a disconnected branch, and
    lookups across the gap fail at runtime with a message that names the
    frames but not the cause.
    """
    # In a scratch directory: view_frames writes frames_<date>.gv and .pdf
    # into its working directory even with yaml_only, and `make test-ros`
    # runs from the package source -- so every run left two files in src/.
    code, out = _run(['timeout', '12', 'ros2', 'run', 'tf2_tools',
                      'view_frames', '--ros-args', '-p', 'yaml_only:=true'],
                     timeout=25, cwd=tempfile.mkdtemp())
    # view_frames prints a YAML map of frame -> parent when yaml_only.
    roots = [line.split(':')[0].strip()
             for line in out.splitlines()
             if line.strip().endswith('parent: ')]
    assert len(roots) <= 1, f'TF has multiple roots: {roots}'


def _home_the_arm():
    """Send the arm to its reference configuration and wait."""
    import time
    send_arm_goal(ROBOT['poses']['home'])
    time.sleep(1.5)


def test_tool0_matches_hand_computed_home_position(robot):
    """
    TF agrees with the value computed by hand from the YAML.

    0.045 + 0.020 + 0.110 + 0.095 + 0.030 = 0.300 m.

    The first term was 0.030 until the CAD showed the base could not hold
    its own pan servo and it grew to 45 mm. The fast-suite copies of this
    number were updated then; this one lives in `make test-ros`, which
    nothing runs automatically, and it stayed wrong until running the
    course's Tier 4 lessons found it. Kept hand-computed anyway: a value
    derived by the code under test would agree with that code by
    construction.

    Homes the arm explicitly. The robot fixture is session-scoped and
    shared, so an earlier test in another module may well have left it
    somewhere else - assuming a pristine pose is how this first failed.

    Only the 3VN arm has a figure to compare against: its links are
    lengths in a YAML that a person can add up. A foreign arm's
    geometry is a chain of rotated offsets exported from CAD, and a
    number "computed by hand" from that would in practice be copied
    from the code under test. The KDL comparison below covers it
    instead.
    """
    if ROBOT['kind'] != 'arm':
        pytest.skip(
            f'{ROBOT["profile"]}: no hand-computed home position exists '
            'for a vendor arm; its kinematics are checked against KDL in '
            'test_our_kinematics_agrees_with_kdl')
    _home_the_arm()
    x, y, z = _tf_translation('base_link', 'tool0')
    assert x == pytest.approx(0.0, abs=1e-3)
    assert y == pytest.approx(0.0, abs=1e-3)
    assert z == pytest.approx(0.300, abs=1e-3)


def test_our_kinematics_agrees_with_kdl(robot):
    """
    Our forward kinematics matches robot_state_publisher's, frame by frame.

    Two independent implementations - ours, and KDL inside RSP - built
    from the same description. Disagreement means one of them is wrong,
    and the test does not need to know which to be worth having.

    Compared away from home, at a pose with every joint somewhere
    different and none at zero. At the reference configuration most of
    the angles are zero and a swapped axis or a wrong sign cancels out;
    the session-scoped robot may well be sitting there when this runs,
    so the test moves it rather than hoping.
    """
    from threevn_control.kinematics import parent_map, tool_position

    target = legal_targets(scale=1.5)
    code, out = send_arm_goal(target)
    assert code == 0, out
    import time
    time.sleep(1.5)

    parents = parent_map(ROBOT['urdf'])
    positions = joint_positions()
    moved = [j for j in ROBOT['arm_joints'] if abs(positions[j]) > 0.05]
    assert moved == ROBOT['arm_joints'], (
        f'only {moved} left zero; the comparison would be at a pose that '
        'hides sign errors')

    for frame in FRAMES:
        ours = np.array(tool_position(parents, positions, target=frame,
                                      base=BASE))
        theirs = np.array(_tf_translation(BASE, frame))
        assert np.allclose(ours, theirs, atol=1e-3), (
            f'{frame} at {positions}: kinematics says {np.round(ours, 4)}, '
            f'TF says {np.round(theirs, 4)}'
        )

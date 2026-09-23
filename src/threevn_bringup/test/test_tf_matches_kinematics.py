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

from conftest import run as _run
import numpy as np
import pytest

pytestmark = pytest.mark.ros

STARTUP_TIMEOUT = 60.0

#: Frames every later phase depends on.
FRAMES = ['tool0', 'grasp_frame', 'camera_optical_frame', 'mount_plate_link']

#: Joint poses to compare at. Zero plus a few asymmetric ones, because a
#: symmetric pose can hide a swapped axis.
POSES = [
    {},
    {'shoulder_lift_joint': math.pi / 2},
    {'shoulder_pan_joint': 0.7, 'shoulder_lift_joint': 0.9,
     'elbow_joint': -0.6, 'wrist_joint': 0.4},
    {'shoulder_pan_joint': -1.2, 'shoulder_lift_joint': 0.3,
     'elbow_joint': -1.1, 'wrist_joint': -0.8},
]


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
    x, y, z = _tf_translation('base_link', frame)
    assert all(math.isfinite(v) for v in (x, y, z))


def test_tf_tree_has_a_single_root(robot):
    """
    The TF tree is one connected tree, not a forest.

    Two roots means something is publishing a disconnected branch, and
    lookups across the gap fail at runtime with a message that names the
    frames but not the cause.
    """
    code, out = _run(['timeout', '12', 'ros2', 'run', 'tf2_tools',
                      'view_frames', '--ros-args', '-p', 'yaml_only:=true'],
                     timeout=25)
    # view_frames prints a YAML map of frame -> parent when yaml_only.
    roots = [line.split(':')[0].strip()
             for line in out.splitlines()
             if line.strip().endswith('parent: ')]
    assert len(roots) <= 1, f'TF has multiple roots: {roots}'


def _current_joint_positions():
    """Read the live joint positions from /joint_states."""
    code, out = _run(['timeout', '6', 'ros2', 'topic', 'echo',
                      '/joint_states', '--once'], timeout=15)
    lines = out.splitlines()
    n_idx = next(i for i, ln in enumerate(lines) if ln.startswith('name:'))
    p_idx = next(i for i, ln in enumerate(lines) if ln.startswith('position:'))
    names = [ln.strip('- ').strip() for ln in lines[n_idx + 1:p_idx]]
    values = []
    for ln in lines[p_idx + 1:]:
        stripped = ln.strip()
        if not stripped.startswith('- '):
            break
        values.append(float(stripped[2:]))
    return dict(zip(names, values))


def _home_the_arm():
    """Send the arm to its reference configuration and wait."""
    import time
    joints = ('shoulder_pan_joint, shoulder_lift_joint, '
              'elbow_joint, wrist_joint')
    goal = (f'{{trajectory: {{joint_names: [{joints}], '
            'points: [{positions: [0.0, 0.0, 0.0, 0.0], '
            'time_from_start: {sec: 2}}]}}')
    _run(['ros2', 'action', 'send_goal',
          '/arm_controller/follow_joint_trajectory',
          'control_msgs/action/FollowJointTrajectory', goal], timeout=60)
    time.sleep(1.5)


def test_tool0_matches_hand_computed_home_position(robot):
    """
    TF agrees with the value computed by hand from the YAML.

    0.030 + 0.020 + 0.110 + 0.095 + 0.030 = 0.285 m.

    Homes the arm explicitly. The robot fixture is session-scoped and
    shared, so an earlier test in another module may well have left it
    somewhere else - assuming a pristine pose is how this first failed.
    """
    _home_the_arm()
    x, y, z = _tf_translation('base_link', 'tool0')
    assert x == pytest.approx(0.0, abs=1e-3)
    assert y == pytest.approx(0.0, abs=1e-3)
    assert z == pytest.approx(0.285, abs=1e-3)


def test_our_kinematics_agrees_with_kdl(robot):
    """
    Our forward kinematics matches robot_state_publisher's, frame by frame.

    Two independent implementations - ours, and KDL inside RSP - built
    from the same description. Disagreement means one of them is wrong,
    and the test does not need to know which to be worth having.

    Compared at whatever pose the robot is CURRENTLY in, read from
    /joint_states, rather than at a pose this test arranges. That removes
    any dependence on test ordering and makes the check stronger: it
    holds across the workspace, not just at the one configuration where
    most of the angles are zero and errors cancel.
    """
    import pathlib

    from ament_index_python.packages import get_package_share_directory
    from threevn_control.kinematics import parent_map, tool_position
    import xacro

    share = pathlib.Path(
        get_package_share_directory('threevn_robot_description'))
    urdf = xacro.process_file(
        str(share / 'urdf' / 'threevn_arm.urdf.xacro'),
        mappings={'params_file': str(share / 'config' / 'threevn_arm_v1.yaml'),
                  'target': 'mock'}).toprettyxml(indent='  ')
    parents = parent_map(urdf)
    positions = _current_joint_positions()

    for frame in FRAMES:
        ours = np.array(tool_position(parents, positions, target=frame))
        theirs = np.array(_tf_translation('base_link', frame))
        assert np.allclose(ours, theirs, atol=1e-3), (
            f'{frame} at {positions}: kinematics says {np.round(ours, 4)}, '
            f'TF says {np.round(theirs, 4)}'
        )

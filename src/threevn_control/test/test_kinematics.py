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
Forward kinematics, checked against hand arithmetic. No ROS, no robot.

Two of these numbers are not the test's own invention: they were computed
by hand from the YAML during Phase 1 and confirmed against a running
robot_state_publisher. Pinning them here means a change to the geometry
that breaks kinematics fails immediately rather than at the next
integration run.
"""

import math
import pathlib

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from threevn_control.kinematics import (
    forward_kinematics,
    parent_map,
    rotation,
    rpy_to_matrix,
    tool_position,
)
import xacro
import yaml

SHARE = pathlib.Path(get_package_share_directory('threevn_robot_description'))
CONFIG_DIR = SHARE / 'config'
XACRO_ENTRY = SHARE / 'urdf' / 'threevn_arm.urdf.xacro'


def _is_arm(path):
    """Arm profiles only: this module walks the ARM chain by name."""
    return yaml.safe_load(path.read_text())['meta']['kind'] == 'arm'


#: Arm-family profiles. The mobile base has its own kinematics -- wheel
#: odometry, not a serial chain -- and asserting `base_link -> tool0` on
#: it is meaningless rather than merely failing.
PROFILES = sorted(p for p in CONFIG_DIR.glob('threevn_*.yaml') if _is_arm(p))


def _expand(profile):
    """Expand a profile to URDF, the same entry point ros2 launch uses."""
    doc = xacro.process_file(
        str(XACRO_ENTRY),
        mappings={'params_file': str(profile), 'target': 'mock'})
    return doc.toprettyxml(indent='  ')


@pytest.fixture(scope='module')
def urdf():
    """Return the expanded URDF for the default profile."""
    return _expand(CONFIG_DIR / 'threevn_arm_v1.yaml')


@pytest.fixture(scope='module')
def config():
    """Return the default robot profile, for raw numbers."""
    return yaml.safe_load((CONFIG_DIR / 'threevn_arm_v1.yaml').read_text())


# -- the primitives ----------------------------------------------------

def test_rotation_by_zero_is_identity():
    """A zero rotation changes nothing."""
    assert np.allclose(rotation((0, 0, 1), 0.0), np.eye(4))


def test_rotation_is_orthonormal():
    """
    A rotation matrix must be orthonormal with determinant +1.

    A determinant of -1 is a reflection, which silently mirrors the robot
    rather than turning it.
    """
    matrix = rotation((0.3, -0.5, 0.8), 1.1)[:3, :3]
    assert np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(matrix) == pytest.approx(1.0, abs=1e-12)


def test_rotation_about_z_by_90_degrees_maps_x_to_y():
    """A known rotation moves a known vector where it should."""
    point = rotation((0, 0, 1), math.pi / 2) @ np.array([1.0, 0, 0, 1.0])
    assert np.allclose(point[:3], [0.0, 1.0, 0.0], atol=1e-12)


def test_rpy_uses_urdf_fixed_axis_order():
    """
    RPY composes as Rz(yaw) @ Ry(pitch) @ Rx(roll), per URDF.

    Applying the three in a different order gives a frame that looks
    almost right, which is far harder to notice than one that is
    obviously wrong.
    """
    roll, pitch, yaw = 0.3, -0.7, 1.2
    expected = (rotation((0, 0, 1), yaw)
                @ rotation((0, 1, 0), pitch)
                @ rotation((1, 0, 0), roll))
    assert np.allclose(rpy_to_matrix(roll, pitch, yaw), expected)


# -- the arm -----------------------------------------------------------

def test_tool0_at_home_matches_hand_arithmetic(urdf):
    """
    At the reference configuration the arm is straight up.

    0.045 (shoulder_pan origin) + 0.020 (shoulder_lift) + 0.110 (upper
    arm) + 0.095 (forearm) + 0.030 (tool0) = 0.300 m, which is what a
    running robot_state_publisher reported in Phase 1.
    """
    x, y, z = tool_position(urdf, {})
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(0.300, abs=1e-9)


def test_tool0_with_shoulder_lift_at_90_degrees(urdf):
    """
    Lifting the shoulder 90 degrees swings the arm to horizontal.

    The chain beyond shoulder_lift (0.110 + 0.095 + 0.030 = 0.235) rotates
    from +Z to +X, leaving 0.045 + 0.020 = 0.065 of height below the
    joint. Verified against TF in Phase 1 as [0.235, 0.000, 0.050].
    """
    x, y, z = tool_position(
        urdf, {'shoulder_lift_joint': math.pi / 2})
    assert x == pytest.approx(0.235, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(0.065, abs=1e-9)


def test_shoulder_pan_rotates_the_whole_arm_about_z(urdf):
    """
    Panning 90 degrees moves a horizontal arm from +X to +Y.

    Checks the two joints compose in the right order; swapping them
    would give the same reach with the wrong bearing.
    """
    x, y, z = tool_position(urdf, {
        'shoulder_pan_joint': math.pi / 2,
        'shoulder_lift_joint': math.pi / 2,
    })
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.235, abs=1e-9)
    assert z == pytest.approx(0.065, abs=1e-9)


def test_reach_is_bounded_by_the_link_lengths(config, urdf):
    """
    No joint combination puts the tool further out than the arm is long.

    A cheap invariant that catches a sign error or a double-counted link
    offset anywhere in the chain, without needing a reference value for
    every pose.
    """
    links = config['links']
    max_reach = (0.045 + 0.020
                 + links['upper_arm_link']['geometry']['z']
                 + links['forearm_link']['geometry']['z']
                 + links['wrist_link']['geometry']['length']
                 + 0.030)
    rng = np.random.default_rng(seed=1)
    for _ in range(200):
        angles = {
            name: rng.uniform(math.radians(spec['limit']['lower_deg']),
                              math.radians(spec['limit']['upper_deg']))
            for name, spec in config['joints'].items()
            if spec['type'] == 'revolute'
        }
        distance = np.linalg.norm(tool_position(urdf, angles))
        assert distance <= max_reach + 1e-9, \
            f'reach {distance:.4f} exceeds {max_reach:.4f} at {angles}'


def test_transforms_stay_valid_across_the_workspace(config, urdf):
    """Every computed transform is a proper rigid-body transform."""
    rng = np.random.default_rng(seed=2)
    for _ in range(100):
        angles = {
            name: rng.uniform(math.radians(spec['limit']['lower_deg']),
                              math.radians(spec['limit']['upper_deg']))
            for name, spec in config['joints'].items()
            if spec['type'] == 'revolute'
        }
        matrix = forward_kinematics(urdf, angles)
        assert np.allclose(matrix[3, :], [0, 0, 0, 1], atol=1e-12)
        block = matrix[:3, :3]
        assert np.allclose(block @ block.T, np.eye(3), atol=1e-9)
        assert np.linalg.det(block) == pytest.approx(1.0, abs=1e-9)


def test_camera_optical_frame_applies_the_rep103_rotation(urdf):
    """
    The optical frame looks along its own +Z.

    REP-103: an optical frame has +z forward, +x right, +y down. The mount
    tilts -0.35 rad about Y, so the camera's forward axis should point
    ahead and downward in base_link - positive X, negative Z.
    """
    matrix = forward_kinematics(
        urdf, {}, target='camera_optical_frame', base='base_link')
    forward = matrix[:3, 2]
    assert forward[0] > 0.5, f'optical +Z should point forward, got {forward}'
    assert forward[2] < 0.0, f'optical +Z should tilt downward, got {forward}'


def test_gripper_finger_slides_along_its_prismatic_axis(urdf):
    """A prismatic joint translates rather than rotates."""
    closed = np.array(tool_position(
        urdf, {}, target='gripper_left_finger_link',
        base='gripper_base_link'))
    opened = np.array(tool_position(
        urdf, {'gripper_left_finger_joint': 0.018},
        target='gripper_left_finger_link', base='gripper_base_link'))
    delta = opened - closed
    assert delta[1] == pytest.approx(0.018, abs=1e-9)
    assert abs(delta[0]) < 1e-12 and abs(delta[2]) < 1e-12


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_fk_works_for_every_profile(profile):
    """
    Kinematics is profile-driven, not hardcoded to one arm.

    The long-reach variant must produce a longer reach purely from its
    YAML, with no change here.
    """
    x, y, z = tool_position(_expand(profile), {})
    assert x == pytest.approx(0.0, abs=1e-9)
    assert z > 0.2, f'{profile.stem}: implausible home height {z}'


def test_long_reach_profile_actually_reaches_further():
    """The variant differs in the way its name claims."""
    default = _expand(CONFIG_DIR / 'threevn_arm_v1.yaml')
    long_reach = _expand(CONFIG_DIR / 'threevn_arm_v1_long_reach.yaml')
    assert (tool_position(long_reach, {})[2]
            > tool_position(default, {})[2] + 0.05)


def test_every_link_reaches_the_base(urdf):
    """
    Every link in the URDF is reachable from base_link.

    This is the check that would have caught the incomplete parent map
    immediately. The earlier YAML-derived version silently omitted joints
    the xacro creates - base_footprint to base_link, the gripper body,
    the camera chain - so whole frames were unreachable and only showed
    up when something asked for them by name.
    """
    import xml.etree.ElementTree as ET

    parents = parent_map(urdf)
    links = {link.get('name') for link in ET.fromstring(urdf).iter('link')}
    unreachable = []
    for link in sorted(links):
        if link in ('base_link', 'base_footprint'):
            continue
        try:
            forward_kinematics(parents, {}, target=link, base='base_link')
        except ValueError:
            unreachable.append(link)
    assert not unreachable, f'not reachable from base_link: {unreachable}'

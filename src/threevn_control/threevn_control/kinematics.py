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
Forward kinematics for the 3VN arm, computed from the profile YAML.

Where the arm's tool tip is, given its joint angles. Built from the same
`config/threevn_arm_v1.yaml` that generates the URDF, so a dimension
changed in one place changes both the simulation and this.

This is deliberately a SECOND implementation. robot_state_publisher
already computes the same transforms through KDL, and
test_fk_matches_tf.py asserts the two agree to a micrometre. Two
independent implementations that agree is real evidence; one
implementation that agrees with itself is not.

It is also useful on its own: a scenario can assert where the gripper
ended up in Cartesian space rather than only which joint angles it
reached, and nothing else in the stack can answer that without a running
robot.

Only forward kinematics. Inverse kinematics is genuinely harder for a
4-DOF arm (the workspace is not fully reachable in position AND
orientation) and belongs with MoveIt rather than a hand-rolled solver.
"""

import math

import numpy as np


def rotation(axis, angle):
    """
    Return the 4x4 homogeneous rotation about a unit axis.

    Rodrigues' formula. Written out rather than pulled from a library so
    the arithmetic is auditable next to the tests that check it.
    """
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0:
        return np.eye(4)
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    d = 1.0 - c
    matrix = np.eye(4)
    matrix[:3, :3] = np.array([
        [c + x * x * d,     x * y * d - z * s, x * z * d + y * s],
        [y * x * d + z * s, c + y * y * d,     y * z * d - x * s],
        [z * x * d - y * s, z * y * d + x * s, c + z * z * d],
    ])
    return matrix


def rpy_to_matrix(roll, pitch, yaw):
    """
    Return the 4x4 rotation for URDF roll-pitch-yaw.

    URDF uses fixed-axis XYZ, applied as R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    Getting this order wrong is a classic source of frames that look
    almost right, so it is stated explicitly and checked against TF.
    """
    return (rotation((0, 0, 1), yaw)
            @ rotation((0, 1, 0), pitch)
            @ rotation((1, 0, 0), roll))


def translation(xyz):
    """Return the 4x4 homogeneous translation."""
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    return matrix


def joint_transform(spec, position=0.0):
    """
    Return the 4x4 transform a joint contributes.

    The fixed origin offset, then the motion the joint itself produces:
    a rotation about its axis for revolute, a slide along it for
    prismatic, and nothing for fixed.
    """
    origin = spec.get('origin', {})
    xyz = origin.get('xyz', [0.0, 0.0, 0.0])
    rpy = origin.get('rpy', [0.0, 0.0, 0.0])
    fixed = translation(xyz) @ rpy_to_matrix(*rpy)

    kind = spec.get('type', 'fixed')
    if kind == 'revolute':
        return fixed @ rotation(spec['axis'], position)
    if kind == 'prismatic':
        axis = np.array(spec['axis'], dtype=float)
        norm = np.linalg.norm(axis)
        if norm:
            axis = axis / norm
        return fixed @ translation(axis * position)
    return fixed


def parent_map(urdf_xml):
    """
    Map child link -> (parent link, origin xyz, origin rpy, type, axis).

    Built from the EXPANDED URDF rather than the profile YAML.

    The YAML describes the arm's joints and named frames, but the xacro
    adds structure of its own - base_footprint to base_link, the gripper
    body to its mount, the camera chain. A parent map derived from YAML
    alone is therefore incomplete by construction, which is how
    camera_optical_frame and then grasp_frame both turned out to be
    unreachable.

    The URDF is still generated from that same YAML, so this remains one
    source of truth; it is simply the complete form of it. And the
    arithmetic below is still entirely our own, so agreement with KDL
    remains real evidence rather than a tautology.
    """
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf_xml)
    parents = {}
    for joint in root.iter('joint'):
        child = joint.find('child')
        parent = joint.find('parent')
        if child is None or parent is None:
            continue
        origin = joint.find('origin')
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get('xyz'):
                xyz = [float(v) for v in origin.get('xyz').split()]
            if origin.get('rpy'):
                rpy = [float(v) for v in origin.get('rpy').split()]
        axis_el = joint.find('axis')
        axis = ([float(v) for v in axis_el.get('xyz').split()]
                if axis_el is not None and axis_el.get('xyz') else [0.0, 0.0, 1.0])
        parents[child.get('link')] = {
            'name': joint.get('name'),
            'parent': parent.get('link'),
            'origin': {'xyz': xyz, 'rpy': rpy},
            'type': joint.get('type', 'fixed'),
            'axis': axis,
        }
    return parents


def forward_kinematics(urdf_xml, joint_positions, target='tool0',
                       base='base_link'):
    """
    Return the 4x4 transform of `target` expressed in `base`.

    `joint_positions` maps joint name to radians (or metres, prismatic).
    Any joint not listed is treated as zero, which is what the robot
    reports at its reference configuration.
    """
    parents = (urdf_xml if isinstance(urdf_xml, dict)
               else parent_map(urdf_xml))

    transforms = []
    node = target
    guard = 0
    while node != base:
        if node not in parents:
            raise ValueError(
                f'{target!r} does not reach {base!r}: stopped at {node!r}')
        spec = parents[node]
        transforms.append(
            joint_transform(spec, float(joint_positions.get(spec['name'], 0.0))))
        node = spec['parent']
        guard += 1
        if guard > 64:
            raise ValueError('kinematic chain is implausibly deep; cycle?')

    result = np.eye(4)
    for matrix in reversed(transforms):
        result = result @ matrix
    return result


def tool_position(urdf_xml, joint_positions, target='tool0', base='base_link'):
    """Return just the (x, y, z) of `target` in `base`."""
    return tuple(
        forward_kinematics(urdf_xml, joint_positions, target, base)[:3, 3])

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


# -- inverse kinematics ------------------------------------------------
#
# Forward kinematics above is general: it walks whatever chain the URDF
# describes. The inverse is NOT, and cannot be. A closed form exists only
# for particular structures, so everything below first checks that this
# robot is the structure it assumes, and refuses if it is not. A closed
# form quietly applied to an arm it does not fit returns joint angles
# that are wrong rather than an error, which is the worst outcome
# available.


class Unreachable(ValueError):
    """Raised when no joint solution puts the tool at the requested pose."""


def joint_limits(urdf_xml):
    """Return joint name -> (lower, upper) in radians, from the URDF."""
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf_xml)
    limits = {}
    for joint in root.iter('joint'):
        limit = joint.find('limit')
        name = joint.get('name')
        if limit is None or name is None:
            continue
        lower, upper = limit.get('lower'), limit.get('upper')
        if lower is not None and upper is not None:
            limits[name] = (float(lower), float(upper))
    return limits


def arm_geometry(urdf_xml, target='grasp_frame', base='base_link'):
    """
    Measure the arm this URDF describes, and check the closed form fits it.

    Returns the four driven joint names, the three link lengths and the
    shoulder height -- all MEASURED from the URDF at its zero
    configuration rather than copied from the YAML. Copying them would
    be a second place the numbers live, and the two would eventually
    disagree; this way changing a link length changes the IK.

    The structure required is a turntable carrying a planar 3R arm:

        joint 1   revolute about Z      (yaw: which way the arm faces)
        joints 2-4 revolute about Y     (parallel: all move in one plane)

    with every link offset along +Z at zero, i.e. the arm points straight
    up in its reference configuration. That is what makes the solution
    below exact. Anything else raises.
    """
    parents = (urdf_xml if isinstance(urdf_xml, dict)
               else parent_map(urdf_xml))

    chain = []
    node = target
    while node != base:
        if node not in parents:
            raise ValueError(f'{target!r} does not reach {base!r}')
        chain.append((node, parents[node]))
        node = parents[node]['parent']
    chain.reverse()

    driven = [(link, spec) for link, spec in chain
              if spec['type'] in ('revolute', 'continuous')]
    if len(driven) != 4:
        raise Unreachable(
            f'closed-form IK needs exactly 4 driven joints between '
            f'{base!r} and {target!r}; this chain has {len(driven)}')

    def axis_is(spec, wanted):
        axis = np.array(spec['axis'], dtype=float)
        norm = np.linalg.norm(axis)
        if not norm:
            return False
        return abs(abs(float(np.dot(axis / norm, wanted))) - 1.0) < 1e-6

    if not axis_is(driven[0][1], np.array([0.0, 0.0, 1.0])):
        raise Unreachable('first joint must turn about Z for this closed form')
    for link, spec in driven[1:]:
        if not axis_is(spec, np.array([0.0, 1.0, 0.0])):
            raise Unreachable(
                f"{spec['name']!r} must turn about Y for this closed form")

    zero = {}
    pos = [np.array(tool_position(parents, zero, target=link, base=base))
           for link, _ in driven]
    tip = np.array(tool_position(parents, zero, target=target, base=base))

    def offset(a, b):
        delta = b - a
        if abs(delta[0]) > 1e-9 or abs(delta[1]) > 1e-9:
            raise Unreachable(
                'links must lie along Z at the zero configuration; '
                f'found offset {np.round(delta, 5).tolist()}')
        return float(delta[2])

    shoulder = pos[1]
    if abs(shoulder[0]) > 1e-9 or abs(shoulder[1]) > 1e-9:
        raise Unreachable('the shoulder must sit on the yaw axis')

    return {
        'joints': [spec['name'] for _, spec in driven],
        'shoulder_z': float(shoulder[2]),
        'a1': offset(pos[1], pos[2]),
        'a2': offset(pos[2], pos[3]),
        'a3': offset(pos[3], tip),
        'limits': (joint_limits(urdf_xml)
                   if not isinstance(urdf_xml, dict) else {}),
    }


def inverse_kinematics(urdf_xml, position, pitch=math.pi, elbow='down',
                       target='grasp_frame', base='base_link',
                       geometry=None, respect_limits=True):
    """
    Return joint angles putting `target` at `position` with the given pitch.

    Four joints, four constraints: three of position and one of
    orientation. `pitch` is the direction the last link points, measured
    from straight up, so pi means straight DOWN -- a top-down grasp,
    which is the useful default for picking things off a table.

    `elbow` selects between the two solutions that reach the same point
    ('up' or 'down'); both are returned by the algebra and only the
    limits distinguish them, so when one violates a limit the other is
    tried before giving up.

    Raises Unreachable, with the distance and the arm's actual reach,
    rather than returning a nearby pose that silently misses.
    """
    geom = geometry or arm_geometry(urdf_xml, target=target, base=base)
    a1, a2, a3 = geom['a1'], geom['a2'], geom['a3']
    names = geom['joints']

    x, y, z = (float(v) for v in position)

    # 1. Yaw is decoupled: the whole planar arm swings to face the point.
    yaw = math.atan2(y, x)

    # 2. Work in that plane. r is distance from the yaw axis, h is height
    #    above the shoulder pivot.
    r = math.hypot(x, y)
    h = z - geom['shoulder_z']

    # 3. Subtract the last link, whose direction `pitch` fixes. What is
    #    left is where the wrist has to be, which is a plain 2R problem.
    rw = r - a3 * math.sin(pitch)
    hw = h - a3 * math.cos(pitch)

    span = math.hypot(rw, hw)
    if span > a1 + a2 + 1e-9 or span < abs(a1 - a2) - 1e-9:
        raise Unreachable(
            f'wrist would need to sit {span * 1000:.0f} mm from the shoulder; '
            f'this arm spans {abs(a1 - a2) * 1000:.0f}-{(a1 + a2) * 1000:.0f} mm '
            f'(reach to the tool is {(a1 + a2 + a3) * 1000:.0f} mm)')

    cos_elbow = (span * span - a1 * a1 - a2 * a2) / (2.0 * a1 * a2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    magnitude = math.acos(cos_elbow)

    orders = [magnitude, -magnitude]
    if elbow == 'up':
        orders.reverse()

    failures = []
    for bend in orders:
        lift = (math.atan2(rw, hw)
                - math.atan2(a2 * math.sin(bend), a1 + a2 * math.cos(bend)))
        wrist = pitch - lift - bend
        solution = dict(zip(names, (yaw, lift, bend, wrist)))
        if not respect_limits:
            return solution
        outside = _outside_limits(solution, geom['limits'])
        if not outside:
            return solution
        failures.append(outside)

    raise Unreachable(
        'the point is within reach but every solution violates a joint '
        'limit: ' + '; '.join(failures))


def _outside_limits(solution, limits):
    """Return a description of the first limit violated, or '' if none."""
    for name, value in solution.items():
        bounds = limits.get(name)
        if bounds is None:
            continue
        lower, upper = bounds
        if not (lower - 1e-6 <= value <= upper + 1e-6):
            return (f'{name} needs {math.degrees(value):+.1f} deg, '
                    f'limit [{math.degrees(lower):+.1f}, '
                    f'{math.degrees(upper):+.1f}]')
    return ''


def reachable(urdf_xml, position, pitch=math.pi, geometry=None, **kwargs):
    """Return True when `inverse_kinematics` can place the tool there."""
    try:
        inverse_kinematics(urdf_xml, position, pitch=pitch,
                           geometry=geometry, **kwargs)
        return True
    except Unreachable:
        return False


#: Approach angles tried when choosing one, steepest first. 180 is
#: straight down; 60 is a shallow, almost-horizontal reach.
PITCH_CANDIDATES = tuple(math.radians(d) for d in range(180, 49, -2))


def feasible_pitch_range(urdf_xml, position, geometry=None,
                         candidates=PITCH_CANDIDATES, **kwargs):
    """
    Return (lowest, highest) approach pitch that reaches `position`, or None.

    Pitch is the one orientation this arm has to spare, and fixing it in
    advance throws that away. Measured on the default arm, a cube resting
    on the ground can be reached across a 54 mm span of radius -- but at
    any SINGLE pitch the span collapses to about 6 mm. Choosing the pitch
    to suit the object, rather than the object to suit the pitch, is the
    difference between a usable workspace and a razor's edge.
    """
    found = [p for p in candidates
             if reachable(urdf_xml, position, pitch=p,
                          geometry=geometry, **kwargs)]
    if not found:
        return None
    return (min(found), max(found))


def choose_pitch(urdf_xml, position, geometry=None, prefer='middle',
                 candidates=PITCH_CANDIDATES, **kwargs):
    """
    Return one approach pitch for `position`, or None if there is none.

    `prefer='middle'` takes the centre of the feasible range, which is
    the choice that tolerates the most error in where the object
    actually is -- and the object's position is an estimate, so that
    margin is the thing being bought. `prefer='steepest'` takes the most
    top-down instead, for reaching into something.
    """
    found = sorted(p for p in candidates
                   if reachable(urdf_xml, position, pitch=p,
                                geometry=geometry, **kwargs))
    if not found:
        return None
    if prefer == 'steepest':
        return found[-1]
    # The MEDIAN of the feasible angles, not the midpoint of their range.
    # The feasible set is not always an interval -- the two elbow
    # branches can leave a gap -- so the midpoint may be a pitch that
    # does not solve at all. Returning a candidate that was actually
    # tested cannot have that failure, which this did have: it handed
    # back 118 degrees from a 102-134 range and the IK then refused it.
    return found[len(found) // 2]

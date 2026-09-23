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
Where the composed robot's mass actually is.

A mobile manipulator can throw itself over. The arm is a large fraction
of the total mass and it moves, so the centre of mass is a function of
the joint angles -- which means "does it tip" is a question about the
whole reachable workspace, not about one pose.

None of this needs a simulator. It is geometry and arithmetic over the
same URDF that runs, and it answers before any hardware exists.
"""

import itertools
import pathlib

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from threevn_control.kinematics import forward_kinematics, parent_map
from urdf_parser_py.urdf import URDF
import xacro
import yaml

SHARE = pathlib.Path(get_package_share_directory('threevn_robot_description'))
MM_XACRO = SHARE / 'urdf' / 'threevn_mobile_manipulator.urdf.xacro'
CONFIG_DIR = SHARE / 'config'

ROOT = 'base_footprint'


def _compositions():
    """Every composed-robot profile shipped in the package."""
    found = []
    for path in sorted(CONFIG_DIR.glob('threevn_*.yaml')):
        doc = yaml.safe_load(path.read_text())
        if doc['meta']['kind'] == 'mobile_manipulator':
            found.append(path)
    return found


#: Parametrizing over these is what makes the long-reach composition more
#: than a file that exists: the stability sweep runs against the arm that
#: puts the same payload furthest out.
COMPOSITIONS = _compositions()


class Assembled:
    """One composed robot, expanded once and reused."""

    def __init__(self, profile):
        self.profile = profile
        self.mm = yaml.safe_load(profile.read_text())
        self.urdf = xacro.process_file(
            str(MM_XACRO),
            mappings={'params_file': str(profile), 'target': 'mock',
                      'prefix': '', 'controllers_file': ''},
        ).toprettyxml(indent='  ')
        self.robot = URDF.from_xml_string(self.urdf)
        self.parents = parent_map(self.urdf)
        self.prefix = self.mm['mount']['prefix']
        self.base_cfg = yaml.safe_load(
            (CONFIG_DIR / self.mm['components']['base']).read_text())


_CACHE = {}


@pytest.fixture(params=COMPOSITIONS, ids=lambda p: p.stem)
def rig(request):
    """Provide an assembled robot per composition profile."""
    key = str(request.param)
    if key not in _CACHE:
        _CACHE[key] = Assembled(request.param)
    return _CACHE[key]


def centre_of_mass(robot, parents, positions):
    """
    Return the whole robot's centre of mass in the root frame, and its mass.

    Each link's inertial origin is expressed in that link's own frame, so
    it has to be carried through the link transform rather than added to
    it. Skipping that step is a quiet way to compute a centre of mass
    that is merely the average of the link ORIGINS.
    """
    total = 0.0
    moment = np.zeros(3)
    for link in robot.links:
        if link.inertial is None or not link.inertial.mass:
            continue
        transform = forward_kinematics(parents, positions,
                                       target=link.name, base=ROOT)
        offset = np.array(link.inertial.origin.position
                          if link.inertial.origin else [0.0, 0.0, 0.0])
        point = transform @ np.append(offset, 1.0)
        total += link.inertial.mass
        moment += link.inertial.mass * point[:3]
    return moment / total, total


def support_polygon(base_cfg):
    """
    Return the wheel contact points, in the root frame.

    The wheels touch the ground directly below their axles, so the
    contact patch is the axle's (x, y). Four of them make the rectangle
    the robot may not put its centre of mass outside.
    """
    return [tuple(spec['position'])
            for name, spec in base_cfg['joints'].items()
            if 'wheel' in name]


def _limits(rig):
    """Return the arm's joint limits, prefixed as the composition names them."""
    limits = {}
    for name in ('shoulder_pan_joint', 'shoulder_lift_joint',
                 'elbow_joint', 'wrist_joint'):
        joint = rig.robot.joint_map[f'{rig.prefix}{name}']
        limits[f'{rig.prefix}{name}'] = (joint.limit.lower, joint.limit.upper)
    return limits


# -- the mount is where the profile says ------------------------------

def test_the_arm_plate_sits_on_the_chassis_plate(rig):
    """
    The arm's mounting face coincides with the base's mounting point.

    The composition derives this offset from the arm's own
    mount_plate_link rather than restating it, so this checks the
    derivation: if the arm's base plate ever changes thickness, the arm
    should still sit ON the chassis rather than sunk into it or floating
    above it.
    """
    mount = forward_kinematics(rig.parents, {}, target='arm_mount_link',
                               base=ROOT)
    plate = forward_kinematics(rig.parents, {},
                               target=f'{rig.prefix}mount_plate_link',
                               base=ROOT)
    # The profile offsets the arm in the plane deliberately; only the
    # vertical coincidence is derived, so only it is asserted here.
    assert plate[2, 3] == pytest.approx(mount[2, 3], abs=1e-9), (
        f'the arm plate sits at z={plate[2, 3]:.4f} but the chassis '
        f'mount is at z={mount[2, 3]:.4f}'
    )


def test_the_arm_sits_where_the_profile_says(rig):
    """The declared in-plane offset reaches the URDF unmodified."""
    mount = forward_kinematics(rig.parents, {}, target='arm_mount_link',
                               base=ROOT)
    pad = forward_kinematics(rig.parents, {},
                             target=f'{rig.prefix}mount_pad_link', base=ROOT)
    assert pad[0, 3] - mount[0, 3] == pytest.approx(
        rig.mm['mount']['x'], abs=1e-9)
    assert pad[1, 3] - mount[1, 3] == pytest.approx(
        rig.mm['mount']['y'], abs=1e-9)


def test_nothing_is_below_the_floor(rig):
    """
    No link origin ends up under the ground plane at the home pose.

    Phase 10 shipped a chassis buried 20 mm into the floor, and the
    simulator hid it because the spawn height lifted the robot clear.
    Composition moves everything, so the check is worth repeating on the
    assembled robot.
    """
    for link in rig.robot.links:
        z = forward_kinematics(rig.parents, {}, target=link.name,
                               base=ROOT)[2, 3]
        assert z > -1e-6, f'{link.name} is {z:.4f} m below the floor'


def test_the_arm_reaches_past_its_own_chassis(rig):
    """
    The arm can reach beyond the front of the base.

    A manipulator that cannot reach past the vehicle carrying it can only
    work on its own roof. This is the cheapest possible check that
    mounting the arm has not made it useless, and it is the constraint
    that pulls against the tipping margin: every millimetre of reach is a
    millimetre of leverage.
    """
    chassis = rig.base_cfg['links']['base_link']['geometry']['x'] / 2.0
    best = max(
        forward_kinematics(rig.parents, dict(zip(_limits(rig), combo)),
                           target=f'{rig.prefix}grasp_frame', base=ROOT)[0, 3]
        for combo in itertools.product(
            *(np.linspace(lo, hi, 5) for lo, hi in _limits(rig).values()))
    )
    assert best > chassis, (
        f'the gripper reaches x={best:.3f} m, not past the chassis front '
        f'edge at {chassis:.3f} m'
    )


# -- can it tip itself over -------------------------------------------

def test_the_centre_of_mass_starts_inside_the_wheelbase(rig):
    """At rest, with the arm folded, the robot is obviously stable."""
    com, _ = centre_of_mass(rig.robot, rig.parents, {})
    contacts = support_polygon(rig.base_cfg)
    front = max(p[0] for p in contacts)
    rear = min(p[0] for p in contacts)
    assert rear < com[0] < front, (
        f'centre of mass at x={com[0]:.4f} is outside the wheelbase '
        f'({rear:.3f} to {front:.3f}) with the arm at home'
    )


def test_the_robot_cannot_tip_itself_over(rig):
    """
    THE WORST POSE IN THE WHOLE WORKSPACE KEEPS THE ROBOT DOWN.

    Not one pose: a sweep of the arm's joint limits, carrying the
    declared payload at the grasp frame, looking for the configuration
    that pushes the centre of mass furthest toward a wheel. The support
    rectangle is NARROWER front-to-back (wheels at x = +/-0.070) than
    side-to-side (y = +/-0.085), so reaching forward is the risk.

    Mounting the arm behind centre buys margin against reaching forward
    and spends it at the rear, which is why the sweep checks all four
    edges rather than the one that looked dangerous.

    A margin is required rather than bare containment. A centre of mass
    exactly over an axle is already unrecoverable in practice: braking, a
    floor seam or the arm's own momentum finishes it.
    """
    contacts = support_polygon(rig.base_cfg)
    front = max(p[0] for p in contacts)
    rear = min(p[0] for p in contacts)
    left = max(p[1] for p in contacts)
    right = min(p[1] for p in contacts)

    # 30% of the half-wheelbase, kept back from the edge.
    margin = 0.30 * (front - rear) / 2.0
    limits = _limits(rig)
    payload = rig.mm['payload']['max_mass']

    worst = None
    for combo in itertools.product(*(np.linspace(lo, hi, 5)
                                     for lo, hi in limits.values())):
        positions = dict(zip(limits, combo))
        com, mass = centre_of_mass(rig.robot, rig.parents, positions)

        # The payload rides at the grasp frame and moves the result.
        grasp = forward_kinematics(rig.parents, positions,
                                   target=f'{rig.prefix}grasp_frame',
                                   base=ROOT)[:3, 3]
        com = (com * mass + grasp * payload) / (mass + payload)

        for value, edge, label in (
            (com[0], front, 'front'), (-com[0], -rear, 'rear'),
            (com[1], left, 'left'), (-com[1], -right, 'right'),
        ):
            slack = edge - value
            if worst is None or slack < worst[0]:
                worst = (slack, label, positions, com)

    slack, label, positions, com = worst
    pose = {k.replace(rig.prefix, ''): round(float(v), 3)
            for k, v in positions.items()}
    assert slack > margin, (
        f'{rig.profile.stem} tips over its {label} wheels: centre of mass '
        f'reaches ({com[0]:+.4f}, {com[1]:+.4f}) with '
        f'{payload * 1000:.0f} g held, leaving {slack * 1000:+.1f} mm of a '
        f'required {margin * 1000:.1f} mm margin. Worst pose: {pose}. Move '
        f'the arm (mount.x), carry less, or widen the base.'
    )

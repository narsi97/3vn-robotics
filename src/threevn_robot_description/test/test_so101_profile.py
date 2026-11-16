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
The SO-101 profile, and the description it produces.

Two halves, and the split matters.

The first half reads only so101_follower.yaml. It runs everywhere,
including in CI, where the upstream description has not been fetched.

The second half needs vendor/so101/so101.urdf. Where that is missing
each test SKIPS and says why, with the command that fixes it. A skip is
the right answer on a machine that has not fetched the description and
the wrong one to leave unreported, so `make test` prints skip reasons
and `make test-ros PROFILE=so101_follower` cannot run without the file
at all.
"""
import math
import xml.etree.ElementTree as ET

from conftest import CONFIG_DIR
import pytest
from threevn_robot_description import describe as describing
from urdf_parser_py.urdf import URDF
import yaml

PROFILE = CONFIG_DIR / 'so101_follower.yaml'
CFG = yaml.safe_load(PROFILE.read_text())

FETCHED = (describing.vendor_root() / CFG['description']['urdf']).exists()
needs_description = pytest.mark.skipif(
    not FETCHED,
    reason=('the SO-101 description has not been fetched; run '
            '`make so101-fetch`. It is Apache-2.0 upstream and is not '
            'committed to this repository.'))

#: A pose may come this close to a joint limit and no closer, radians.
#: Measured on the 3VN arm: a pose 2 degrees from a limit did not
#: arrive, because an arm at the edge of its travel cannot hold itself
#: there.
LIMIT_MARGIN = math.radians(5.0)


# -- the profile on its own --------------------------------------------

def test_it_declares_itself_a_vendor_arm():
    """The kind is what routes it to the overlay instead of to xacro."""
    assert CFG['meta']['kind'] in describing.VENDOR_KINDS
    assert CFG['meta']['model'] == PROFILE.stem
    assert CFG['meta']['dof'] == len(CFG['arm']['joints'])


def test_it_restates_no_kinematics():
    """
    No link, origin or axis appears in the overlay.

    Upstream's URDF is the kinematic truth. A length repeated here
    would be a second copy that could disagree with the first, with
    nothing to say which was right.
    """
    assert 'links' not in CFG and 'frames' not in CFG
    for name, joint in CFG['joints'].items():
        assert set(joint) <= {'servo', 'id'}, (
            f'{name} declares {sorted(set(joint) - {"servo", "id"})}; '
            'geometry and limits belong to the upstream URDF')


def test_every_pose_has_one_position_per_arm_joint():
    """Checkable without the description, so checked without it."""
    assert 'home' in CFG['poses']
    for name, positions in CFG['poses'].items():
        assert len(positions) == len(CFG['arm']['joints']), name


def test_every_commanded_joint_has_a_servo_and_a_unique_bus_id():
    """
    Arm joints and the gripper each name a servo and an id of their own.

    Two joints sharing an id is two servos answering one packet. On the
    bus that is a collision; here it is a comparison.
    """
    commanded = describing.controlled_joints(CFG)
    assert set(commanded) == set(CFG['joints'])
    ids = [CFG['joints'][name]['id'] for name in commanded]
    assert sorted(ids) == list(range(1, len(commanded) + 1))
    for name in commanded:
        assert CFG['joints'][name]['servo'] in CFG['servos']


def test_servo_figures_say_where_they_came_from():
    """Every servo declares a provenance, as every 3VN mass does."""
    for name, servo in CFG['servos'].items():
        assert servo['provenance'] in {'measured', 'datasheet', 'estimated'}, name
        assert servo['stall_torque_nm'] > 0 and servo['no_load_speed_rads'] > 0


def test_limits_never_exceed_what_the_servo_can_deliver():
    """
    Derived joint limits stay at or below the datasheet figures.

    A derating above 1.0 would let the controller command torque the
    hardware cannot produce: the motion succeeds in Gazebo and stalls
    on the bench.
    """
    for name in CFG['joints']:
        effort, velocity = describing.joint_limits_for(CFG, name)
        servo = CFG['servos'][CFG['joints'][name]['servo']]
        assert 0 < effort <= servo['stall_torque_nm'], name
        assert 0 < velocity <= servo['no_load_speed_rads'], name


def test_the_hardware_target_is_declared_before_it_exists():
    """
    `feetech` has a plugin name, an endpoint, a baud rate and a loop rate.

    The interface is not written yet. Fixing its name and parameters
    now is what makes writing it an addition to one package instead of
    a change to everything that launches the robot.
    """
    assert describing.targets(PROFILE) == ['mock', 'gz', 'feetech']
    feetech = CFG['hardware']['feetech']
    assert '/' in feetech['plugin']
    assert feetech['transport'] in {'serial', 'tcp'}
    assert feetech['baud'] == 1000000
    assert feetech['control_rate_hz'] > 0


# -- against the fetched description -----------------------------------

@needs_description
@pytest.mark.parametrize('target', ['mock', 'gz', 'feetech'])
def test_it_describes_for_every_target(target):
    """The real overlay on the real URDF parses, anchored to the world."""
    robot = URDF.from_xml_string(describing.describe(PROFILE, target=target))
    assert robot.name == 'so101_follower'
    assert robot.get_root() == 'world'
    for joint in describing.controlled_joints(CFG):
        assert joint in robot.joint_map, f'upstream has no joint {joint!r}'


@needs_description
def test_the_tool_and_base_frames_exist_upstream():
    """The frames the profile names are links in upstream's URDF."""
    robot = URDF.from_xml_string(describing.describe(PROFILE))
    assert CFG['arm']['base_frame'] in robot.link_map
    assert CFG['arm']['tool_frame'] in robot.link_map


@needs_description
def test_every_pose_is_inside_the_limits_with_margin():
    """
    No named pose asks a joint to sit at the edge of its travel.

    Inside the limits, and LIMIT_MARGIN away from them. The client
    refuses a pose outside the limits outright; one just inside is
    accepted and then not reached.
    """
    facts = describing.robot_facts(PROFILE)
    for name, positions in facts['poses'].items():
        for joint, position in zip(facts['arm_joints'], positions):
            lower, upper = facts['limits'][joint]
            assert lower + LIMIT_MARGIN <= position <= upper - LIMIT_MARGIN, (
                f'pose {name!r} sends {joint} to '
                f'{math.degrees(position):+.1f} deg; its travel is '
                f'{math.degrees(lower):+.1f} to {math.degrees(upper):+.1f}')


@needs_description
def test_the_gripper_opens_and_closes_inside_its_travel():
    """Open and closed are angles the jaw joint can reach."""
    facts = describing.robot_facts(PROFILE)
    lower, upper = facts['limits'][facts['gripper_joint']]
    assert lower <= facts['gripper']['closed'] < facts['gripper']['open'] <= upper


@needs_description
def test_upstreams_placeholder_limits_are_replaced():
    """
    No commanded joint is left with upstream's effort="10".

    Ten newton-metres is about five times what an STS3215 stalls at. A
    simulated arm that strong lifts what the real one cannot.
    """
    root = ET.fromstring(describing.describe(PROFILE, target='gz'))
    commanded = set(describing.controlled_joints(CFG))
    seen = set()
    for joint in root.iter('joint'):
        limit = joint.find('limit')
        if joint.get('name') in commanded and limit is not None:
            seen.add(joint.get('name'))
            assert float(limit.get('effort')) < 2.0, joint.get('name')
            assert float(limit.get('velocity')) < 5.0, joint.get('name')
    assert seen == commanded


@needs_description
def test_no_servo_joint_is_a_frictionless_hinge():
    """
    Every commanded joint carries damping and friction into the simulator.

    Found by running it. Without them, closing the jaw with the gripper
    pointing down ran it into its hard stop at the velocity limit, and
    it never opened again -- while `pick_and_place` went on passing,
    because its release step checked nothing.
    """
    root = ET.fromstring(describing.describe(PROFILE, target='gz'))
    commanded = set(describing.controlled_joints(CFG))
    for joint in root.findall('joint'):
        if joint.get('name') not in commanded:
            continue
        dynamics = joint.find('dynamics')
        assert dynamics is not None, joint.get('name')
        assert float(dynamics.get('damping')) > 0, joint.get('name')


@needs_description
def test_the_gripper_travel_stays_clear_of_the_hard_stops():
    """
    Open and closed are INSIDE the jaw's limits, not on them.

    A joint parked exactly on a hard stop in the simulator can stay
    there. The named travel is what the client and the teleop use, so
    it keeps a margin from both ends.
    """
    facts = describing.robot_facts(PROFILE)
    lower, upper = facts['limits'][facts['gripper_joint']]
    assert facts['gripper']['closed'] - lower >= LIMIT_MARGIN
    assert upper - facts['gripper']['open'] >= LIMIT_MARGIN


@needs_description
def test_no_full_resolution_mesh_is_left_in_the_physics():
    """
    Every collision is a primitive unless the profile asked for a mesh.

    This is the difference between a real-time factor of 0.12 and one
    of 0.95 on CPU physics. The visuals keep upstream's meshes.
    """
    policy = CFG['description']['collision']
    wants_mesh = {link for link, mode in (policy.get('links') or {}).items()
                  if mode == 'mesh'}
    root = ET.fromstring(describing.describe(PROFILE, target='gz'))
    visuals = 0
    for link in root.iter('link'):
        visuals += len(link.findall('visual/geometry/mesh'))
        if link.get('name') in wants_mesh:
            continue
        assert link.findall('collision/geometry/mesh') == [], link.get('name')
    assert visuals > 0, 'the visual meshes should be untouched'

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
The overlay that turns somebody else's URDF into a robot this stack runs.

Tested against a small stand-in (fixtures/toy_arm.urdf), not against the
SO-101 itself: that description is fetched, not committed, and a test
that needs the network to run is a test that will be skipped in exactly
the place it matters. The stand-in has the properties the overlay
exists for -- no world link, a relative mesh path, placeholder limits --
with numbers small enough to check by hand.

test_so101_profile.py then checks the real profile against the real
description, when it has been fetched.
"""
import hashlib
import pathlib
import xml.etree.ElementTree as ET

import pytest
from threevn_robot_description import describe as describing
from urdf_parser_py.urdf import URDF

FIXTURES = pathlib.Path(__file__).parent / 'fixtures'
TOY = FIXTURES / 'toy_arm.yaml'
TOY_URDF = FIXTURES / 'toy_arm.urdf'


@pytest.fixture(autouse=True)
def vendor_is_the_fixtures_directory(monkeypatch):
    """Point the vendor root at the fixtures, for every test here."""
    monkeypatch.setenv('THREEVN_VENDOR_DIR', str(FIXTURES))


def _root(target='mock', controllers=''):
    return ET.fromstring(
        describing.describe(TOY, target=target, controllers=controllers))


# -- what the overlay adds ---------------------------------------------

def test_the_arm_is_anchored_to_the_world():
    """
    The description's root is `world`, joined rigidly to the arm's base.

    Upstream has no world link, so its base is a free body: spawned as
    shipped, the arm falls over the first time a joint moves.
    """
    robot = URDF.from_xml_string(describing.describe(TOY))
    assert robot.get_root() == 'world'
    anchor = robot.joint_map['world_to_base_link']
    assert anchor.type == 'fixed'
    assert (anchor.parent, anchor.child) == ('world', 'base_link')


def test_the_robot_is_named_after_its_profile():
    """Spawned and reported under OUR name for it, not upstream's file name."""
    assert _root().get('name') == 'toy_arm'


def test_mesh_paths_are_absolute_and_exist():
    """
    Every mesh is a file:// URL to a file that is really there.

    `assets/x.stl` works for a tool handed the FILE and fails for one
    handed the STRING -- which is how the simulator's spawner and
    robot_state_publisher both get the description.
    """
    meshes = [m.get('filename') for m in _root().iter('mesh')]
    assert meshes, 'the fixture is supposed to have a mesh'
    for name in meshes:
        assert name.startswith('file:///'), name
        assert pathlib.Path(name[len('file://'):]).is_file(), name


def test_limits_are_derived_from_the_servo_not_taken_from_upstream():
    """
    Effort and velocity come from the datasheet through the derating.

    stall 2.0 N.m x 0.5 = 1.0, no-load 4.0 rad/s x 0.25 = 1.0. Upstream
    says 10 and 10 for every joint, which nobody measured.
    """
    for joint in _root().iter('joint'):
        limit = joint.find('limit')
        if limit is None:
            continue
        assert float(limit.get('effort')) == pytest.approx(1.0)
        assert float(limit.get('velocity')) == pytest.approx(1.0)


def test_position_limits_are_upstreams_untouched():
    """The overlay changes what the motor can do, never where the joint stops."""
    limits = {j.get('name'): j.find('limit')
              for j in _root().iter('joint') if j.find('limit') is not None}
    assert (limits['j_b'].get('lower'), limits['j_b'].get('upper')) == \
        ('-0.5', '1.5')


def test_servo_joints_get_the_friction_a_gearbox_has():
    """
    Every servo-driven joint gains <dynamics>; nothing else does.

    Upstream's joints are frictionless hinges. In the simulator that is
    not a detail: a jaw with no friction, closed while gravity helped
    it along, sat at its velocity limit all the way into its hard stop
    -- and the physics engine does not drive a joint that is on a limit.

    The fixed joints are left alone, and so are the <joint> elements
    inside a <transmission>, which are references and not joints.
    """
    root = _root()
    dynamics = {j.get('name'): j.find('dynamics')
                for j in root.findall('joint')}
    for name in ('j_a', 'j_b', 'j_jaw'):
        assert dynamics[name] is not None, name
        assert float(dynamics[name].get('damping')) == pytest.approx(0.05)
        assert float(dynamics[name].get('friction')) == pytest.approx(0.01)
    assert dynamics['tip_joint'] is None
    assert dynamics['world_to_base_link'] is None
    for transmission in root.iter('transmission'):
        for reference in transmission.iter('joint'):
            assert reference.find('dynamics') is None


def test_the_upstream_file_is_never_modified():
    """Describing it, on every target, leaves vendor's copy byte-identical."""
    before = hashlib.sha256(TOY_URDF.read_bytes()).hexdigest()
    for target in ('mock', 'gz', 'feetech'):
        describing.describe(TOY, target=target)
    assert hashlib.sha256(TOY_URDF.read_bytes()).hexdigest() == before


# -- the hardware seam, for a robot we did not design ------------------

def _control(root):
    blocks = root.findall('ros2_control')
    assert len(blocks) == 1, 'exactly one ros2_control block'
    return blocks[0]


@pytest.mark.parametrize('target,plugin', [
    ('mock', 'mock_components/GenericSystem'),
    ('gz', 'gz_ros2_control/GazeboSimSystem'),
    ('feetech', 'toy_hardware/ToySystemInterface'),
])
def test_each_target_selects_exactly_its_plugin(target, plugin):
    """One plugin per description, chosen by the target and nothing else."""
    plugins = [p.text for p in _control(_root(target)).iter('plugin')]
    assert plugins == [plugin]


def test_only_the_simulator_target_mentions_the_simulator():
    """
    A description bound for real hardware contains no Gazebo at all.

    The same property test_ros2_control_targets.py asserts for our own
    arm: the seam is real only if nothing leaks through it.
    """
    for target in ('mock', 'feetech'):
        text = describing.describe(TOY, target=target)
        assert '<gazebo' not in text and 'gz_ros2_control' not in text, target
    assert '<gazebo' in describing.describe(TOY, target='gz')


def test_gz_target_is_told_where_the_controllers_are():
    """The simulator hosts controller_manager, so it gets the parameters file."""
    root = _root('gz', controllers='/somewhere/controllers.yaml')
    assert root.find('gazebo/plugin/parameters').text == \
        '/somewhere/controllers.yaml'


def test_every_commanded_joint_has_interfaces_mirroring_its_limits():
    """Arm joints then the gripper, each with position limits from the URDF."""
    joints = _control(_root()).findall('joint')
    assert [j.get('name') for j in joints] == ['j_a', 'j_b', 'j_jaw']
    j_b = joints[1]
    params = {p.get('name'): p.text
              for p in j_b.find('command_interface').findall('param')}
    assert params == {'min': '-0.5', 'max': '1.5'}
    assert [s.get('name') for s in j_b.findall('state_interface')] == \
        ['position', 'velocity']


def test_only_real_hardware_is_told_the_servo_ids():
    """
    Bus ids reach the hardware target and no other.

    A simulator has no bus. An id parameter there would be one more
    thing that differs between descriptions for no reason.
    """
    def ids(target):
        return {j.get('name'): p.text
                for j in _control(_root(target)).findall('joint')
                for p in j.findall('param') if p.get('name') == 'id'}

    assert ids('feetech') == {'j_a': '1', 'j_b': '2', 'j_jaw': '3'}
    assert ids('mock') == {} and ids('gz') == {}


def test_hardware_parameters_come_from_the_profile():
    """Endpoint, baud and rate are passed through; the plugin name is not."""
    hardware = _control(_root('feetech')).find('hardware')
    params = {p.get('name'): p.text for p in hardware.findall('param')}
    assert params == {'transport': 'serial', 'endpoint': '/dev/null',
                      'baud': '1000000', 'control_rate_hz': '50'}


def test_an_unknown_target_is_refused_with_the_ones_that_exist():
    """`target:=esp32` on a robot with no ESP32 says what it does have."""
    with pytest.raises(describing.DescriptionUnavailable) as excinfo:
        describing.describe(TOY, target='esp32')
    assert 'feetech' in str(excinfo.value) and 'mock' in str(excinfo.value)
    assert describing.targets(TOY) == ['mock', 'gz', 'feetech']


# -- collision geometry -------------------------------------------------

def test_a_box_collision_is_the_meshs_bounding_box_in_the_same_place():
    """
    `box` replaces a mesh collision with that mesh's bounds.

    The fixture mesh spans x 0..0.1, y -0.01..0.01, z 0..0.02, so the
    box is 0.1 x 0.02 x 0.02 centred at (0.05, 0, 0.01) in the mesh
    frame. The collision sits at z = 0.5 turned a quarter turn about z,
    which carries that centre to (0, 0.05, 0.01): the box belongs at
    (0, 0.05, 0.51).
    """
    link = next(lk for lk in _root().iter('link') if lk.get('name') == 'link_a')
    collision = link.find('collision')
    assert collision.find('geometry/mesh') is None
    size = [float(v) for v in collision.find('geometry/box').get('size').split()]
    assert size == pytest.approx([0.1, 0.02, 0.02])
    xyz = [float(v) for v in collision.find('origin').get('xyz').split()]
    assert xyz == pytest.approx([0.0, 0.05, 0.51], abs=1e-5)
    assert collision.find('origin').get('rpy') == '0 0 1.5708'


def test_a_link_can_opt_out_of_collision_altogether():
    """`none` removes a link's collisions; the fixture asks it of link_b."""
    link = next(lk for lk in _root().iter('link') if lk.get('name') == 'link_b')
    assert link.findall('collision') == []


def test_visuals_keep_their_meshes():
    """Simplifying what the physics sees never changes what a person sees."""
    link = next(lk for lk in _root().iter('link') if lk.get('name') == 'link_a')
    assert link.find('visual/geometry/mesh') is not None


def test_binary_and_ascii_stl_give_the_same_bounds(tmp_path):
    """
    The bounds reader handles both STL encodings.

    Upstream's meshes are binary; the fixture is ASCII so that it can be
    read in a diff. Same triangles, same six numbers.
    """
    import struct
    triangles = [((0.0, -0.01, 0.0), (0.1, 0.01, 0.0), (0.1, -0.01, 0.02)),
                 ((0.0, 0.01, 0.02), (0.1, 0.01, 0.0), (0.0, -0.01, 0.0))]
    blob = b'\0' * 80 + struct.pack('<I', len(triangles))
    for triangle in triangles:
        blob += struct.pack('<3f', 0.0, 0.0, 1.0)
        for vertex in triangle:
            blob += struct.pack('<3f', *vertex)
        blob += struct.pack('<H', 0)
    binary = tmp_path / 'toy_link_binary.stl'
    binary.write_bytes(blob)

    ascii_bounds = describing.stl_bounds(FIXTURES / 'assets' / 'toy_link.stl')
    binary_bounds = describing.stl_bounds(binary)
    for ours, theirs in zip(ascii_bounds, binary_bounds):
        assert ours == pytest.approx(theirs)
    assert ascii_bounds[0] == pytest.approx((0.0, -0.01, 0.0))
    assert ascii_bounds[1] == pytest.approx((0.1, 0.01, 0.02))


# -- refusing, with a reason --------------------------------------------

def _broken(tmp_path, mutate):
    """Write a copy of the toy profile with one thing wrong."""
    import yaml
    cfg = yaml.safe_load(TOY.read_text())
    mutate(cfg)
    path = tmp_path / 'broken.yaml'
    path.write_text(yaml.safe_dump(cfg))
    return path


def test_a_missing_description_says_how_to_get_it(tmp_path, monkeypatch):
    """
    No vendor URDF is a message about fetching, not a FileNotFoundError.

    This is the first thing a student on a fresh clone will see.
    """
    monkeypatch.setenv('THREEVN_VENDOR_DIR', str(tmp_path))
    with pytest.raises(describing.DescriptionUnavailable) as excinfo:
        describing.describe(TOY)
    assert 'make toy-fetch' in str(excinfo.value)


def test_a_pose_of_the_wrong_length_is_refused_before_anything_moves(tmp_path):
    """
    A three-number pose for a two-joint arm fails at DESCRIPTION time.

    The original failure -- "expected 5 joint positions, got 4" -- came
    from a running robot. The same mistake in a profile now stops the
    launch, with the pose named.
    """
    profile = _broken(
        tmp_path, lambda cfg: cfg['poses'].update(ready=[0.3, -0.3, 0.1]))
    with pytest.raises(describing.DescriptionUnavailable) as excinfo:
        describing.describe(profile)
    assert "'ready'" in str(excinfo.value)


def test_a_joint_upstream_does_not_have_is_refused_by_name(tmp_path):
    """A profile naming a joint the URDF lacks says which, and what exists."""
    profile = _broken(
        tmp_path, lambda cfg: cfg['arm'].update(joints=['j_a', 'j_c']))
    with pytest.raises(describing.DescriptionUnavailable) as excinfo:
        describing.describe(profile)
    assert "'j_c'" in str(excinfo.value) and 'j_b' in str(excinfo.value)


def test_a_vendor_description_cannot_be_prefixed():
    """Its frame names are upstream's; a prefix would invent robots."""
    with pytest.raises(describing.DescriptionUnavailable):
        describing.describe(TOY, prefix='arm_')


# -- and it is still a URDF ---------------------------------------------

@pytest.mark.parametrize('target', ['mock', 'gz', 'feetech'])
def test_the_result_parses_and_carries_its_semantics(target):
    """The overlaid description parses, and says what the profile says."""
    text = describing.describe(TOY, target=target)
    robot = URDF.from_xml_string(text)
    assert {'j_a', 'j_b', 'j_jaw'} <= set(robot.joint_map)

    semantics = describing.parse_semantics(text)
    assert semantics['arm_joints'] == ['j_a', 'j_b']
    assert semantics['tool_frame'] == 'tip'
    assert semantics['gripper'] == {'joint': 'j_jaw', 'open': 0.7,
                                    'closed': 0.0}
    assert semantics['poses'] == {'home': [0.0, 0.0], 'ready': [0.3, -0.3]}


def test_robot_facts_are_read_from_the_description():
    """What the integration tests are told matches what was described."""
    facts = describing.robot_facts(TOY)
    assert facts['kind'] == 'vendor_arm'
    assert facts['hardware'] == ['toy_arm']
    assert facts['arm_joints'] == ['j_a', 'j_b']
    assert facts['gripper_joint'] == 'j_jaw'
    assert facts['gripper_followers'] == []
    assert facts['limits']['j_b'] == (-0.5, 1.5)

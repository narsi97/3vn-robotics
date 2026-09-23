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
The arm, on the base.

Composition means the two components are unchanged and still work alone.
That is not a comment, it is the thing these tests check: if composing
them required editing either, Phase 11 would have merged two robots into
a third one to maintain rather than assembled two that already existed.
"""

import xml.etree.ElementTree as ET

from conftest import CONFIG_DIR, ENTRY_POINTS
import pytest
from urdf_parser_py.urdf import URDF
import xacro
import yaml

MM_PROFILE = CONFIG_DIR / 'threevn_mm_v1.yaml'

WHEEL_JOINTS = ['front_left_wheel_joint', 'front_right_wheel_joint',
                'rear_left_wheel_joint', 'rear_right_wheel_joint']

ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint',
              'elbow_joint', 'wrist_joint']


@pytest.fixture(scope='module')
def mm():
    """Provide the composition profile."""
    return yaml.safe_load(MM_PROFILE.read_text())


@pytest.fixture(scope='module')
def prefix(mm):
    """Provide the arm's prefix, as the profile declares it."""
    return mm['mount']['prefix']


def expand(entry, profile, target='mock'):
    """Expand any entry point against any profile."""
    return xacro.process_file(
        str(entry),
        mappings={'params_file': str(profile), 'target': target,
                  'prefix': '', 'controllers_file': ''},
    ).toprettyxml(indent='  ')


@pytest.fixture(scope='module')
def composed():
    """Expand the composed robot, memoised per target."""
    cache = {}

    def _expand(target='mock'):
        if target not in cache:
            cache[target] = expand(
                ENTRY_POINTS['mobile_manipulator'], MM_PROFILE, target)
        return cache[target]

    return _expand


# -- both subsystems are present ---------------------------------------

def test_every_wheel_survives_composition(composed):
    """The base keeps all four wheels, unprefixed."""
    robot = URDF.from_xml_string(composed())
    for joint in WHEEL_JOINTS:
        assert joint in robot.joint_map, f'{joint} missing from the composition'


def test_every_arm_joint_survives_composition(composed, prefix):
    """The arm keeps all four joints, under its prefix."""
    robot = URDF.from_xml_string(composed())
    for joint in ARM_JOINTS:
        assert f'{prefix}{joint}' in robot.joint_map, \
            f'{prefix}{joint} missing from the composition'


def test_the_gripper_survives_composition(composed, prefix):
    """Including the mimic finger, which is the fiddliest part to carry over."""
    robot = URDF.from_xml_string(composed())
    left = robot.joint_map.get(f'{prefix}gripper_left_finger_joint')
    right = robot.joint_map.get(f'{prefix}gripper_right_finger_joint')
    assert left is not None and right is not None
    assert right.mimic is not None, 'the right finger lost its mimic'
    assert right.mimic.joint == f'{prefix}gripper_left_finger_joint', (
        f'the mimic points at {right.mimic.joint!r}, which is not the '
        f'prefixed left finger -- a prefix that is applied to link names '
        f'but not to mimic targets produces a URDF that parses and a '
        f'gripper that does not work'
    )


# -- the base owns the robot -------------------------------------------

def test_the_base_owns_the_navigation_frames(composed):
    """
    base_link and base_footprint are the BASE's, unprefixed.

    REP-105 and nav2 both name base_link. In a composed robot there can
    be only one, and it has to be the thing that moves along the floor --
    not the arm's own base plate, which is bolted to it.
    """
    robot = URDF.from_xml_string(composed())
    names = {link.name for link in robot.links}
    assert {'base_footprint', 'base_link'} <= names
    assert robot.joint_map['base_joint'].child == 'base_link'


def test_the_arm_is_entirely_prefixed(composed, prefix):
    """
    Nothing of the arm arrives unprefixed.

    Both descriptions declare an `imu_link`. Two links with one name is
    not a parse error: urdf_parser_py builds a dict, so the second
    silently wins and the tree still looks fine. The prefix is what keeps
    them apart, and it has to cover everything.
    """
    arm_only = {'shoulder_link', 'upper_arm_link', 'forearm_link',
                'wrist_link', 'tool0', 'grasp_frame', 'gripper_base_link',
                'camera_link', 'camera_optical_frame', 'mount_plate_link'}
    names = {link.name for link in URDF.from_xml_string(composed()).links}
    leaked = arm_only & names
    assert not leaked, f'arm links arrived unprefixed: {sorted(leaked)}'
    for name in arm_only:
        assert f'{prefix}{name}' in names, f'{prefix}{name} missing'


def test_both_imu_frames_survive(composed, prefix):
    """The collision that the prefix exists to prevent, asserted directly."""
    names = {link.name for link in URDF.from_xml_string(composed()).links}
    assert 'imu_link' in names, "the base's imu_link was lost"
    assert f'{prefix}imu_link' in names, "the arm's imu_link was lost"


# -- one robot, one controller manager ---------------------------------

def test_one_controller_manager_hosts_both_subsystems(composed):
    """
    Two <ros2_control> blocks, ONE system plugin.

    Both component descriptions used to emit the gz_ros2_control system
    plugin themselves, which was invisible while each robot had one
    subsystem. Composed, that is two controller managers in one simulator
    process, both named `controller_manager`, both loading the same
    controllers file.
    """
    xml = composed(target='gz')
    assert xml.count('GazeboSimROS2ControlPlugin') == 1, (
        'the composed robot must host exactly one controller_manager'
    )
    assert xml.count('<ros2_control ') == 2, (
        'expected one ros2_control block per subsystem'
    )


def test_the_two_control_blocks_have_distinct_names(composed):
    """Two blocks sharing a name is a controller_manager that loads one."""
    root = ET.fromstring(composed())
    names = [rc.get('name') for rc in root.iter('ros2_control')]
    assert len(names) == len(set(names)), f'duplicate ros2_control names: {names}'


def test_every_commanded_joint_belongs_to_exactly_one_block(composed):
    """
    No joint is claimed by both subsystems.

    Two hardware components offering the same command interface is a
    resource conflict that controller_manager reports at activation time,
    long after the description was written.
    """
    root = ET.fromstring(composed())
    seen = {}
    for rc in root.iter('ros2_control'):
        for joint in rc.iter('joint'):
            name = joint.get('name')
            assert name not in seen, (
                f'{name} is claimed by both {seen[name]!r} and '
                f'{rc.get("name")!r}'
            )
            seen[name] = rc.get('name')


# -- composition did not modify the components -------------------------

@pytest.mark.parametrize('role', ['arm', 'base'])
def test_each_component_still_builds_alone(mm, role):
    """
    THIS IS WHAT MAKES IT COMPOSITION.

    The arm and the base must remain independently buildable. If mounting
    one on the other required editing either, there would now be a third
    robot to maintain instead of two that already worked.
    """
    part = CONFIG_DIR / mm['components'][role]
    kind = yaml.safe_load(part.read_text())['meta']['kind']
    xml = expand(ENTRY_POINTS[kind], part)
    assert '<robot' in xml
    robot = URDF.from_xml_string(xml)
    assert 'base_link' in {link.name for link in robot.links}


def test_mass_is_conserved_across_composition(mm, composed):
    """
    The composed robot weighs what its parts weigh.

    Nothing lost, nothing counted twice. A component silently dropped
    still produces a URDF that parses, spawns and drives -- just a robot
    missing an arm, which is easy to miss in a headless test and obvious
    only in a mass total.
    """
    def total(xml):
        return sum(link.inertial.mass
                   for link in URDF.from_xml_string(xml).links
                   if link.inertial)

    parts = 0.0
    for role in ('arm', 'base'):
        part = CONFIG_DIR / mm['components'][role]
        kind = yaml.safe_load(part.read_text())['meta']['kind']
        parts += total(expand(ENTRY_POINTS[kind], part))

    assert total(composed()) == pytest.approx(parts, rel=1e-9), (
        'the composed robot does not weigh the sum of its components'
    )


def test_the_composition_adds_no_mass_of_its_own(composed, prefix):
    """
    The joining hardware is not modelled, and that is a known omission.

    Whatever bolts the arm to the chassis has mass, and this description
    says it does not. The mount pad is a pure frame. Stated here so the
    omission is deliberate and visible rather than an oversight waiting
    to be discovered when the real robot is heavier than the model.
    """
    robot = URDF.from_xml_string(composed())
    pad = robot.link_map[f'{prefix}mount_pad_link']
    assert pad.inertial is None, (
        'the mount pad now has mass -- if the mounting hardware is being '
        'modelled, say so in the profile and update this test'
    )

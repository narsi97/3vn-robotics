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
describe() is one door, and both kinds of robot come through it.

Every launch file gets its description from describe(). For a robot we
designed that must be exactly what xacro produces -- these tests compare
the two byte for byte, so the new door cannot quietly become a second
way of building our own arm.

The rest is the <threevn_semantics> block: the arm's joints, the
gripper's travel and the named poses, carried in the description so a
client reads them off the robot it is attached to.
"""
import math
import pathlib
import xml.etree.ElementTree as ET

from conftest import (
    ALL_PROFILES,
    ARM_PROFILES,
    COMPOSED_PROFILES,
    ENTRY_POINTS,
    TARGETS,
)
import pytest
from threevn_robot_description import describe as describing
import yaml


@pytest.mark.parametrize('target', TARGETS)
@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
def test_describe_is_xacro_for_our_own_robots(expanded, profile, target):
    """
    For a robot we designed, describe() returns xacro's output unchanged.

    Accepts the profile's NAME, which is what a launch argument carries,
    and is compared against the conftest's expansion of the PATH.
    """
    assert describing.describe(profile.stem, target=target) == \
        expanded(profile, target)


def test_entry_points_agree_with_the_test_harness():
    """
    describe.py and conftest.py map each family to the same xacro.

    The mapping exists in more than one place because shell and Python
    callers both need it. Two copies that disagreed would expand one
    profile as two different robots, depending on who asked.
    """
    urdf = describing.share() / 'urdf'
    assert {kind: urdf / name
            for kind, name in describing.ENTRY_POINTS.items()} == ENTRY_POINTS


def test_an_unknown_profile_names_the_ones_that_exist():
    """A typo in PROFILE= gets a list to choose from, not a traceback."""
    with pytest.raises(describing.DescriptionUnavailable) as excinfo:
        describing.describe('threevn_arm_v9')
    assert 'threevn_arm_v1' in str(excinfo.value)


# -- <threevn_semantics> -----------------------------------------------

@pytest.mark.parametrize('profile', ARM_PROFILES, ids=lambda p: p.stem)
def test_semantics_carry_the_profile(expanded, profile):
    """What the profile says about arm, gripper and poses reaches the URDF."""
    cfg = yaml.safe_load(profile.read_text())
    semantics = describing.parse_semantics(expanded(profile))

    assert semantics is not None, 'no <threevn_semantics> in the description'
    assert semantics['profile'] == cfg['meta']['model']
    assert semantics['arm_joints'] == cfg['arm']['joints']
    assert semantics['base_frame'] == cfg['arm']['base_frame']
    assert semantics['tool_frame'] == cfg['arm']['tool_frame']
    assert semantics['gripper'] == {
        'joint': cfg['gripper']['joint'],
        'open': float(cfg['gripper']['open']),
        'closed': float(cfg['gripper']['closed']),
    }
    assert semantics['poses'] == {
        name: [float(v) for v in positions]
        for name, positions in cfg['poses'].items()
    }
    # Order is part of the contract: the scenarios visit them by name,
    # but a person reading `make urdf` reads them top to bottom.
    assert list(semantics['poses']) == list(cfg['poses'])


@pytest.mark.parametrize('profile', COMPOSED_PROFILES, ids=lambda p: p.stem)
def test_semantics_follow_the_arm_onto_a_base(expanded, profile):
    """
    A composed robot's semantics name its PREFIXED joints and frames.

    The mobile manipulator calls the arm's first joint
    `arm_shoulder_pan_joint`. A pose block still saying
    `shoulder_pan_joint` would describe joints this robot does not have.
    """
    semantics = describing.parse_semantics(expanded(profile))
    assert semantics is not None
    urdf_joints = {j.get('name')
                   for j in ET.fromstring(expanded(profile)).iter('joint')}
    for joint in semantics['arm_joints'] + [semantics['gripper']['joint']]:
        assert joint in urdf_joints, f'{joint} is not a joint of this robot'
    assert all(j.startswith('arm_') for j in semantics['arm_joints'])


def _limits(urdf):
    limits = {}
    for joint in ET.fromstring(urdf).iter('joint'):
        limit = joint.find('limit')
        if limit is not None and limit.get('lower') is not None:
            limits[joint.get('name')] = (float(limit.get('lower')),
                                         float(limit.get('upper')))
    return limits


@pytest.mark.parametrize('profile', ARM_PROFILES + COMPOSED_PROFILES,
                         ids=lambda p: p.stem)
def test_every_named_pose_fits_the_arm(expanded, profile):
    """
    Each pose has one position per arm joint, inside that joint's limit.

    THE TEST THAT WOULD HAVE CAUGHT IT. The poses were four-element
    lists in the client, and the first arm with five joints found that
    out at runtime, at home(), in a simulator. Here it is a length
    comparison in the fast suite.

    The limit half matters as much: the client refuses an out-of-limit
    pose with LimitViolation, so a pose outside its own robot's limits
    is a scenario that can never pass.
    """
    urdf = expanded(profile)
    semantics = describing.parse_semantics(urdf)
    limits = _limits(urdf)
    assert semantics['poses'], 'a robot with no named poses cannot be homed'
    assert 'home' in semantics['poses'], 'every scenario starts with home()'
    for name, positions in semantics['poses'].items():
        assert len(positions) == len(semantics['arm_joints']), (
            f'pose {name!r} has {len(positions)} positions for '
            f'{len(semantics["arm_joints"])} joints')
        for joint, position in zip(semantics['arm_joints'], positions):
            lower, upper = limits[joint]
            assert lower <= position <= upper, (
                f'pose {name!r} sends {joint} to {position:+.3f} rad '
                f'({math.degrees(position):+.1f} deg), outside '
                f'[{lower:+.3f}, {upper:+.3f}]')


@pytest.mark.parametrize('profile', ARM_PROFILES + COMPOSED_PROFILES,
                         ids=lambda p: p.stem)
def test_gripper_travel_is_inside_its_joint_limits(expanded, profile):
    """
    Open and closed are positions the gripper joint can reach, off its stops.

    STRICTLY inside, by half a millimetre at least: in Gazebo a joint on
    its stop is held by the stop and not driven, and a travel equal to
    the limits is how the 3VN gripper stopped moving. Checked on the
    composed robots too, where the arm's joints are prefixed.
    """
    urdf = expanded(profile)
    gripper = describing.parse_semantics(urdf)['gripper']
    lower, upper = _limits(urdf)[gripper['joint']]
    assert lower + 0.0005 <= gripper['closed'] < gripper['open'] <= upper - 0.0005


@pytest.mark.parametrize('target', TARGETS)
def test_semantics_do_not_depend_on_the_target(expanded, target):
    """
    The same robot says the same thing about itself on every target.

    Named poses that differed between mock and hardware would be the
    seam leaking: a scenario proven in simulation would be driving the
    physical arm somewhere else.
    """
    profile = ARM_PROFILES[0]
    assert describing.parse_semantics(expanded(profile, target)) == \
        describing.parse_semantics(expanded(profile, 'mock'))


def test_controllers_file_defaults_and_can_be_named(tmp_path):
    """A profile runs under controllers.yaml unless it names another."""
    assert describing.controllers_file('threevn_arm_v1') == 'controllers.yaml'
    fixture = pathlib.Path(__file__).parent / 'fixtures' / 'toy_arm.yaml'
    assert describing.controllers_file(fixture) == 'toy_controllers.yaml'

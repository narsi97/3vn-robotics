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
No named pose puts the tool through the bench.

A pose inside every joint limit can still be a collision: joint limits
bound each joint alone, and the bench is a constraint on all of them
together. The mock target has no bench and will pass any pose; the
simulator has one, and a pose that reaches into it fails there as a
trajectory that never arrives, which reads as a control problem.

So the height of the tool at every named pose is computed from the
description, with the same forward kinematics that is checked against
KDL in `make test-ros`. No robot needs to be running.
"""

import xml.etree.ElementTree as ET

import pytest
from threevn_control.kinematics import parent_map, tool_position
from threevn_robot_description import describe as describing

#: The lowest a tool frame may be at a named pose, metres above the bench.
CLEARANCE = 0.02


def _profiles():
    """Return every single-arm profile that can be described right now."""
    found = []
    for path in sorted((describing.share() / 'config').glob('*.yaml')):
        cfg = describing.load_profile(path)
        if 'arm' not in cfg:
            continue
        marks = []
        if cfg['meta']['kind'] in describing.VENDOR_KINDS and not (
                describing.vendor_root() / cfg['description']['urdf']).exists():
            marks.append(pytest.mark.skip(
                reason=f'{path.stem}: description not fetched; run '
                       f'`{cfg["description"].get("fetch", "its fetch script")}`'))
        found.append(pytest.param(path, id=path.stem, marks=marks))
    return found


def _root_link(urdf):
    """Return the one link that is nobody's child."""
    root = ET.fromstring(urdf)
    links = {link.get('name') for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    (name,) = links - children
    return name


@pytest.mark.parametrize('profile', _profiles())
def test_every_named_pose_keeps_the_tool_above_the_bench(profile):
    """The tool frame is at least CLEARANCE above the floor at every pose."""
    facts = describing.robot_facts(profile)
    parents = parent_map(facts['urdf'])
    floor = _root_link(facts['urdf'])
    assert facts['poses'], 'nothing to check'
    for name, positions in facts['poses'].items():
        joints = dict(zip(facts['arm_joints'], positions))
        height = tool_position(parents, joints, target=facts['tool_frame'],
                               base=floor)[2]
        assert height >= CLEARANCE, (
            f'{profile.stem}: pose {name!r} puts {facts["tool_frame"]} at '
            f'{height * 1000:.0f} mm, under the {CLEARANCE * 1000:.0f} mm '
            'clearance')


def test_the_check_can_fail():
    """
    A pose reaching below the floor is caught.

    The 3VN arm folded forward and down: shoulder a little past level,
    wrist turned under.
    """
    facts = describing.robot_facts('threevn_arm_v1')
    parents = parent_map(facts['urdf'])
    floor = _root_link(facts['urdf'])
    joints = dict(zip(facts['arm_joints'], [0.0, 2.0, 0.0, 1.0]))
    height = tool_position(parents, joints, target=facts['tool_frame'],
                           base=floor)[2]
    assert height < CLEARANCE

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

"""Structural invariants of the kinematic tree."""
import xml.etree.ElementTree as ET

from conftest import ALL_PROFILES, ARM_PROFILES, profile_kind, REQUIRED_FRAMES
import pytest
from urdf_parser_py.urdf import URDF


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
def test_no_duplicate_link_names(expanded, profile):
    """
    Checked on RAW XML, deliberately.

    urdf_parser_py builds a dict keyed by name, so a duplicated link
    silently overwrites the first and the parsed tree looks perfectly
    healthy. The duplicate is only visible before parsing. This is a real
    trap, not a hypothetical one.
    """
    names = [link.get('name') for link in ET.fromstring(expanded(profile=profile)).iter('link')]
    dupes = sorted({n for n in names if names.count(n) > 1})
    assert not dupes, f'duplicate link names: {dupes}'


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
def test_exactly_one_root(expanded, profile):
    robot = URDF.from_xml_string(expanded(profile=profile))
    children = {j.child for j in robot.joints}
    roots = sorted(link.name for link in robot.links if link.name not in children)
    assert roots == ['base_footprint'], f'expected one root, got {roots}'


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
def test_no_orphan_links(expanded, profile):
    """
    Every link is reachable from the root.

    An unreachable link leaves a hole in TF that only shows up as a
    missing transform at runtime.
    """
    robot = URDF.from_xml_string(expanded(profile=profile))
    adjacency = {}
    for joint in robot.joints:
        adjacency.setdefault(joint.parent, []).append(joint.child)

    seen, stack = set(), ['base_footprint']
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        stack.extend(adjacency.get(node, []))

    all_links = {link.name for link in robot.links}
    assert seen == all_links, f'unreachable links: {sorted(all_links - seen)}'


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
def test_required_frames_exist(expanded, profile):
    """
    The frames downstream code names by hand are present.

    Each family has its own set: the arm's TCP and optical frames, the
    base's `arm_mount_link`. These are the names Phase 11 composes
    against, so losing one breaks mounting rather than parsing.
    """
    required = REQUIRED_FRAMES[profile_kind(profile)]
    robot = URDF.from_xml_string(expanded(profile=profile))
    present = {link.name for link in robot.links}
    assert required <= present, f'missing: {sorted(required - present)}'


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
def test_moving_links_have_visual_and_collision(expanded, profile):
    """
    Links with mass have both visual and collision geometry.

    Pure frames legitimately have neither. Anything with mass must have
    both, or it is invisible in RViz or passes through obstacles.
    """
    robot = URDF.from_xml_string(expanded(profile=profile))
    for link in robot.links:
        if link.inertial is None:
            continue
        assert link.visuals, f'{link.name} has mass but no <visual>'
        assert link.collisions, f'{link.name} has mass but no <collision>'


@pytest.mark.parametrize('profile', ARM_PROFILES, ids=lambda p: p.stem)
def test_dof_count_matches_declared(expanded, profile):
    """
    The URDF has exactly as many revolute joints as the config declares.

    The config declares `dof: 4`, so the URDF must have four actuated
    revolute joints. Not three, not five.
    """
    import yaml

    cfg = yaml.safe_load(profile.read_text())
    robot = URDF.from_xml_string(expanded(profile=profile))
    revolute = [j for j in robot.joints if j.type == 'revolute']
    assert len(revolute) == cfg['meta']['dof'], (
        f'declared dof={cfg["meta"]["dof"]} but found {len(revolute)} '
        f'revolute joints: {[j.name for j in revolute]}'
    )

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
Joint limits in the URDF equal the limits in the YAML.

This is NOT tautological, which is the whole point of storing revolute
limits in degrees: the value crosses a deg->rad conversion on its way into
the URDF, and again into the ros2_control command interface. Those are two
independent conversion sites, and they are exactly where a bug lives.
"""
import math

# Arm family only: these read degree-keyed revolute keys that a wheeled base
# does not have. The base asserts its equivalents, against `motors`
# and continuous joints, in test_base_description.py.
from conftest import PROFILES
import pytest
from urdf_parser_py.urdf import URDF
import yaml


def _cfg(profile):
    return yaml.safe_load(profile.read_text())


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_joint_limits_match_config(expanded, profile):
    cfg = _cfg(profile)
    robot = URDF.from_xml_string(expanded(profile=profile))

    for name, spec in cfg['joints'].items():
        joint = robot.joint_map.get(name)
        assert joint is not None, f'{name} missing from the URDF entirely'
        assert joint.limit is not None, f'{name} has no <limit>'

        if spec['type'] == 'revolute':
            lower = math.radians(spec['limit']['lower_deg'])
            upper = math.radians(spec['limit']['upper_deg'])
        else:
            lower = spec['limit']['lower']
            upper = spec['limit']['upper']

        assert joint.limit.lower == pytest.approx(lower, abs=1e-9), \
            f'{name}: lower limit drifted from config'
        assert joint.limit.upper == pytest.approx(upper, abs=1e-9), \
            f'{name}: upper limit drifted from config'
        assert joint.limit.effort == pytest.approx(spec['limit']['effort'])
        assert joint.limit.velocity == pytest.approx(spec['limit']['velocity'])


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_limits_are_ordered(expanded, profile):
    robot = URDF.from_xml_string(expanded(profile=profile))
    for joint in robot.joints:
        if joint.type in ('revolute', 'prismatic'):
            assert joint.limit.lower < joint.limit.upper, \
                f'{joint.name}: limits inverted ({joint.limit.lower} !< {joint.limit.upper})'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_initial_position_is_within_limits(expanded, profile):
    """
    Initial joint positions lie within their own limits.

    A robot that starts outside its own limits is an immediate controller
    fault on startup, and a confusing one.
    """
    cfg = _cfg(profile)
    for name, spec in cfg['joints'].items():
        if 'mimic' in spec:
            continue
        if spec['type'] == 'revolute':
            initial = math.radians(spec['initial_position_deg'])
            lower = math.radians(spec['limit']['lower_deg'])
            upper = math.radians(spec['limit']['upper_deg'])
        else:
            initial = spec['initial_position']
            lower, upper = spec['limit']['lower'], spec['limit']['upper']
        assert lower <= initial <= upper, \
            f'{name}: initial position {initial} outside [{lower}, {upper}]'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_ros2_control_limits_agree_with_urdf_limits(expanded, profile):
    """
    The limit is stated TWICE -- in <limit> and in the.

    <command_interface> min/max params. They must not diverge, or the
    controller will happily command a pose the URDF forbids.

    The gripper is the exception, on purpose: it is commanded through
    its travel, inside its stops. The next test holds it to that.
    """
    import xml.etree.ElementTree as ET

    xml = expanded(profile=profile, target='mock')
    robot = URDF.from_xml_string(xml)
    rc = next(ET.fromstring(xml).iter('ros2_control'))
    gripper = _cfg(profile)['gripper']['joint']

    for joint_el in rc.iter('joint'):
        name = joint_el.get('name')
        cmd = joint_el.find("command_interface[@name='position']")
        assert cmd is not None, f'{name}: no position command interface'
        params = {p.get('name'): float(p.text) for p in cmd.findall('param')}
        assert 'min' in params and 'max' in params, f'{name}: missing min/max'
        if name == gripper:
            continue

        limit = robot.joint_map[name].limit
        assert params['min'] == pytest.approx(limit.lower, abs=1e-9), \
            f'{name}: command_interface min disagrees with <limit> lower'
        assert params['max'] == pytest.approx(limit.upper, abs=1e-9), \
            f'{name}: command_interface max disagrees with <limit> upper'


#: How far inside its hard stops the gripper's travel must stay, in the
#: joint's own unit (metres: this gripper is prismatic). Half of what the
#: profiles ship, so the margin can be tuned but not removed.
GRIPPER_STOP_CLEARANCE = 0.0005


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_the_gripper_is_commanded_through_its_travel_not_onto_its_stops(
        expanded, profile):
    """
    The gripper's command range is its travel, and the travel clears its stops.

    In Gazebo a joint at or past its stop is held there by the stop and
    is not driven by its servo. This gripper's travel used to BE its
    limits: it spawned on the lower stop and was commanded onto both, and
    whether it moved came down to rounding. Measured at -5.4e-21 m, every
    command stalled. A real servo wants the same gap, since one driven
    into a hard stop is a stalled servo. See docs/manipulation.md.

    ros2_control enforces the tighter of <limit> and these min/max, so a
    command range equal to the travel means no command can reach a stop.
    """
    import xml.etree.ElementTree as ET

    cfg = _cfg(profile)
    gripper = cfg['gripper']
    xml = expanded(profile=profile, target='mock')
    limit = URDF.from_xml_string(xml).joint_map[gripper['joint']].limit
    rc = next(ET.fromstring(xml).iter('ros2_control'))
    joint_el = next(j for j in rc.iter('joint') if j.get('name') == gripper['joint'])
    cmd = joint_el.find("command_interface[@name='position']")
    params = {p.get('name'): float(p.text) for p in cmd.findall('param')}

    assert params['min'] == pytest.approx(gripper['closed'], abs=1e-9), \
        'the gripper is commanded below gripper.closed'
    assert params['max'] == pytest.approx(gripper['open'], abs=1e-9), \
        'the gripper is commanded beyond gripper.open'
    assert gripper['closed'] - limit.lower >= GRIPPER_STOP_CLEARANCE, (
        f"gripper.closed {gripper['closed']} is on or near its hard stop "
        f'{limit.lower}: a finger closed onto its stop is not driven')
    assert limit.upper - gripper['open'] >= GRIPPER_STOP_CLEARANCE, (
        f"gripper.open {gripper['open']} is on or near its hard stop "
        f'{limit.upper}: a finger opened onto its stop is not driven')

    initial = float(joint_el.find(
        "state_interface[@name='position']/param[@name='initial_value']").text)
    assert gripper['closed'] <= initial <= gripper['open'], (
        f'the gripper starts at {initial}, outside its travel: it spawns '
        'on or beyond the point it is meant to be driven from')

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
    """
    import xml.etree.ElementTree as ET

    xml = expanded(profile=profile, target='mock')
    robot = URDF.from_xml_string(xml)
    rc = next(ET.fromstring(xml).iter('ros2_control'))

    for joint_el in rc.iter('joint'):
        name = joint_el.get('name')
        cmd = joint_el.find("command_interface[@name='position']")
        assert cmd is not None, f'{name}: no position command interface'
        params = {p.get('name'): float(p.text) for p in cmd.findall('param')}
        assert 'min' in params and 'max' in params, f'{name}: missing min/max'

        limit = robot.joint_map[name].limit
        assert params['min'] == pytest.approx(limit.lower, abs=1e-9), \
            f'{name}: command_interface min disagrees with <limit> lower'
        assert params['max'] == pytest.approx(limit.upper, abs=1e-9), \
            f'{name}: command_interface max disagrees with <limit> upper'

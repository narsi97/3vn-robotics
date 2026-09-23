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
The hardware-abstraction seam.

This is the most important file in Phase 1. The spec's central
non-negotiable is that the application must not be coupled to the ESP32 or
to the simulator. These assertions are that requirement, made
machine-checkable: if any of them fail, the decoupling has been broken.
"""
import xml.etree.ElementTree as ET

from conftest import ALL_PROFILES, EXPECTED_PLUGIN, TARGETS
import pytest


def _plugins(xml):
    root = ET.fromstring(xml)
    return [
        p.text.strip()
        for rc in root.iter('ros2_control')
        for p in rc.iter('plugin')
        if p.text
    ]


@pytest.mark.parametrize('target', TARGETS)
def test_exactly_one_hardware_plugin_per_target(expanded, target):
    found = _plugins(expanded(target=target))
    assert found == [EXPECTED_PLUGIN[target]], (
        f'target={target!r} produced {found!r}, expected '
        f'[{EXPECTED_PLUGIN[target]!r}]'
    )


@pytest.mark.parametrize('target', ['mock', 'esp32'])
def test_non_sim_targets_never_reference_gazebo(expanded, target):
    """
    THIS ASSERTION IS THE ARCHITECTURE.

    A real-robot build must not mention Gazebo anywhere -- not a plugin,
    not a <gazebo> tag. If it does, deploying to hardware would drag the
    simulator along and 'the application does not care which it talks to'
    would be false.
    """
    xml = expanded(target=target)
    assert 'gz_ros2_control' not in xml, f'{target} build references gz_ros2_control'
    assert '<gazebo' not in xml, f'{target} build contains a <gazebo> tag'
    assert 'GazeboSim' not in xml


def test_sim_target_never_references_hardware(expanded):
    """And the converse: a simulation build must not reach for a serial port."""
    xml = expanded(target='gz')
    assert 'Esp32SystemInterface' not in xml
    assert '/dev/tty' not in xml


def test_gz_target_emits_the_controller_manager_plugin(expanded):
    """
    The gz target emits the controller_manager system plugin.

    Gazebo hosts controller_manager in its own process. Without this
    plugin, `make sim` starts and silently has no controllers.
    """
    xml = expanded(target='gz')
    assert 'gz_ros2_control-system' in xml
    assert 'GazeboSimROS2ControlPlugin' in xml


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
@pytest.mark.parametrize('target', TARGETS)
def test_seam_holds_for_every_profile(expanded, profile, target):
    """The seam must not depend on which robot profile is loaded."""
    found = _plugins(expanded(profile=profile, target=target))
    assert found == [EXPECTED_PLUGIN[target]]


def test_mimic_joint_is_not_independently_commanded(expanded):
    """
    The right finger follows the left through URDF <mimic>. Giving it.

    its own command interface would fight the mimic and is a classic
    gripper bug.
    """
    root = ET.fromstring(expanded(target='mock'))
    rc = next(root.iter('ros2_control'))
    commanded = {j.get('name') for j in rc.iter('joint')}
    assert 'gripper_left_finger_joint' in commanded
    assert 'gripper_right_finger_joint' not in commanded, (
        'the mimic joint must not have its own command interface'
    )


def test_description_never_resolves_a_package_that_depends_on_it(expanded):
    """
    The description expands without any sibling package installed.

    It once embedded `$(find threevn_bringup)/config/controllers.yaml`,
    which inverted the dependency direction - threevn_bringup depends on
    this package, not the reverse. A fully sourced workspace resolved it
    anyway, so it only surfaced when CI built this package in isolation
    and xacro raised PackageNotFoundError.

    Every target must expand from this package alone.
    """
    for target in TARGETS:
        xml = expanded(target=target)
        assert 'threevn_bringup' not in xml, (
            f'target={target} embeds a path into threevn_bringup; pass it '
            'as the controllers_file argument instead'
        )
        assert 'threevn_sim' not in xml
        assert 'threevn_control' not in xml


def test_the_gz_target_accepts_a_controllers_file(expanded):
    """When supplied, the path reaches the Gazebo plugin block."""
    xml = expanded(target='gz', controllers_file='/tmp/controllers.yaml')
    assert '<parameters>/tmp/controllers.yaml</parameters>' in xml


def test_the_gz_target_omits_parameters_when_none_is_given(expanded):
    """
    Emit no <parameters> tag at all when the argument is empty.

    An empty <parameters></parameters> would be worse than none: Gazebo
    would try to load a controllers file named "".
    """
    xml = expanded(target='gz')
    assert '<parameters>' not in xml

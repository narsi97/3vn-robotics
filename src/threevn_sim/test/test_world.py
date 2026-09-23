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
Fast checks on the simulation assets. No simulator required.

These run in `make test`; the Gazebo integration tests live in
test_gz_spawn.py and run only under `make test-sim`.
"""

import pathlib
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

SHARE = pathlib.Path(get_package_share_directory('threevn_sim'))
WORLDS = sorted((SHARE / 'worlds').glob('*.sdf'))


def test_at_least_one_world_is_shipped():
    """The package installs its worlds where Gazebo can find them."""
    assert WORLDS, f'no .sdf worlds installed under {SHARE / "worlds"}'


@pytest.mark.parametrize('world', WORLDS, ids=lambda p: p.stem)
def test_world_is_well_formed_sdf(world):
    """Each world parses as XML and declares an sdf root with a world."""
    root = ET.fromstring(world.read_text())
    assert root.tag == 'sdf', f'{world.name}: root element is {root.tag!r}'
    assert root.find('world') is not None, f'{world.name}: no <world>'


@pytest.mark.parametrize('world', WORLDS, ids=lambda p: p.stem)
def test_world_declares_a_fixed_physics_step(world):
    """
    Physics uses an explicit step size.

    CI tests depend on this world behaving the same way every run. An
    unspecified step size means Gazebo picks one, which makes gripper
    contacts non-reproducible and turns a flaky test into a mystery.
    """
    physics = ET.fromstring(world.read_text()).find('world/physics')
    assert physics is not None, f'{world.name}: no <physics>'
    step = physics.find('max_step_size')
    assert step is not None, f'{world.name}: no <max_step_size>'
    assert 0 < float(step.text) <= 0.01, \
        f'{world.name}: step {step.text} is too coarse for a small arm'


@pytest.mark.parametrize('world', WORLDS, ids=lambda p: p.stem)
def test_world_loads_the_required_systems(world):
    """
    Physics, UserCommands and SceneBroadcaster are all present.

    Omitting SceneBroadcaster is the classic one: the simulation runs
    correctly and nothing can see it, because no client ever receives the
    scene.
    """
    text = world.read_text()
    for system in ('gz-sim-physics-system',
                   'gz-sim-user-commands-system',
                   'gz-sim-scene-broadcaster-system'):
        assert system in text, f'{world.name}: missing {system}'


def test_bridge_config_is_valid_and_bridges_the_clock():
    """
    The bridge config parses, and /clock is bridged.

    /clock is the one non-optional bridge. Without it the controllers run
    on wall time while the simulator runs on simulation time, and every
    trajectory finishes at the wrong moment for reasons that are very hard
    to see from the outside.
    """
    config = SHARE / 'config' / 'gz_bridge.yaml'
    entries = yaml.safe_load(config.read_text())
    assert isinstance(entries, list) and entries, 'bridge config is empty'
    topics = {e['ros_topic_name'] for e in entries}
    assert '/clock' in topics, f'/clock not bridged; have {sorted(topics)}'


def test_joint_states_is_not_bridged():
    """
    Joint state must NOT be bridged from Gazebo.

    gz_ros2_control already publishes it through ros2_control. Bridging it
    as well creates a second publisher of the same topic carrying slightly
    different data, and whichever a subscriber sees first wins.
    """
    entries = yaml.safe_load((SHARE / 'config' / 'gz_bridge.yaml').read_text())
    topics = {e['ros_topic_name'] for e in entries}
    assert '/joint_states' not in topics

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
ROS integration tests against the `mock` target. No simulator.

The middle tier of the pyramid. The fast suite never starts ROS at all;
the Gazebo suite takes two minutes. These sit between: a real ROS graph,
real controllers, real actions, with no physics - which is enough to test
every interface contract in about twenty seconds.

Marked `ros` and run by `make test-ros`.
"""

import math
import time

from conftest import (
    joint_positions,
    legal_targets,
    ROBOT,
    run as _run,
    send_arm_goal,
)
import pytest

pytestmark = pytest.mark.ros

#: Read from the description of the robot under test -- see conftest.
ARM_JOINTS = ROBOT['arm_joints']


# -- node startup ------------------------------------------------------

def test_expected_nodes_are_running(robot):
    """The bringup launch produces the nodes it is supposed to."""
    code, out = _run(['ros2', 'node', 'list'])
    assert code == 0, out
    for node in ('/controller_manager', '/robot_state_publisher'):
        assert node in out, f'{node} not running:\n{out}'


def test_no_duplicate_nodes(robot):
    """
    Each node appears exactly once.

    A duplicate means a previous launch survived, and two
    robot_state_publishers publishing different descriptions is how the
    Gazebo spawn picked up the wrong plugin in Phase 2.
    """
    code, out = _run(['ros2', 'node', 'list'])
    names = [n for n in out.split() if n.startswith('/')]
    dupes = sorted({n for n in names if names.count(n) > 1})
    assert not dupes, f'duplicate nodes: {dupes}'


# -- the public interface ----------------------------------------------

def test_the_documented_topics_exist(robot):
    """/joint_states and /robot_description are published."""
    code, out = _run(['ros2', 'topic', 'list'])
    assert code == 0, out
    for topic in ('/joint_states', '/robot_description', '/tf', '/tf_static'):
        assert topic in out, f'{topic} missing:\n{out}'


def test_the_documented_services_exist(robot):
    """controller_manager exposes the services the abstraction uses."""
    code, out = _run(['ros2', 'service', 'list'])
    assert code == 0, out
    for service in ('/controller_manager/list_controllers',
                    '/controller_manager/switch_controller',
                    '/controller_manager/list_hardware_components'):
        assert service in out, f'{service} missing'


def test_the_documented_actions_exist(robot):
    """
    Both motion actions are advertised.

    These names are the project's public API and are promised identical
    across mock, Gazebo and hardware, so their presence is a contract
    rather than an implementation detail.
    """
    code, out = _run(['ros2', 'action', 'list'])
    assert code == 0, out
    assert '/arm_controller/follow_joint_trajectory' in out
    assert '/gripper_controller/gripper_cmd' in out


def test_joint_states_publishes_every_actuated_joint(robot):
    """
    Every commandable joint appears in /joint_states.

    A mimic joint deliberately does not - it has no ros2_control
    interface - so on a gripper that has one it is excluded here and
    checked through TF instead. A single-jaw gripper has none, and the
    second half of this test then has nothing to assert.
    """
    code, out = _run(['timeout', '6', 'ros2', 'topic', 'echo',
                      '/joint_states', '--once'], timeout=15)
    for joint in ARM_JOINTS + [ROBOT['gripper_joint']]:
        assert joint in out, f'{joint} missing from /joint_states'
    for follower in ROBOT['gripper_followers']:
        assert follower not in out, \
            f'the mimic joint {follower} should not be in /joint_states'


# -- limit enforcement, the Phase 2 finding, now closed ----------------

def test_out_of_limit_command_is_clamped_below_the_seam(robot):
    """
    ros2_control clamps a command that exceeds a joint limit.

    THIS IS THE PHASE 2 FINDING, CLOSED. Measured then: commanding
    shoulder_pan_joint to 180 degrees against a 90 degree limit drove it
    to a full 180 under mock_components, despite the URDF <limit> AND the
    command_interface min/max both declaring otherwise. Gazebo only
    appeared safe because its physics joint has a hard stop.

    `enforce_command_limits: true` on controller_manager makes
    ResourceManager clamp instead - BELOW the hardware seam, so it holds
    for mock, Gazebo and the ESP32 alike rather than depending on the
    backend.

    Deliberately sent straight to the action, bypassing RobotClient: the
    client's own validation is a separate layer and would mask whether
    this one works.

    The joint is the robot's first, and the limit is its own: a quarter
    turn past wherever this arm's pan stops.
    """
    joint = ARM_JOINTS[0]
    limit = ROBOT['limits'][joint][1]
    zeros = [0.0] * len(ARM_JOINTS)

    send_arm_goal(zeros)
    time.sleep(1.0)

    code, out = send_arm_goal([limit + math.pi / 2] + zeros[1:])
    assert code == 0, out
    time.sleep(1.5)

    actual = joint_positions()[joint]
    assert actual <= limit + 0.01, (
        f'{joint} reached {actual:.5f} rad '
        f'({math.degrees(actual):.1f} deg) against a limit of {limit:.5f} rad '
        '- enforce_command_limits is not working'
    )
    # And it got THERE, not nowhere. Without this, a controller that
    # dropped the goal outright would pass a test about clamping.
    assert actual >= limit - 0.05, (
        f'{joint} stopped at {actual:.5f} rad, well short of its '
        f'{limit:.5f} rad limit - the command was not clamped to the '
        'limit, it was dropped'
    )


def test_in_limit_command_is_not_clamped(robot):
    """
    A legal command is tracked exactly.

    The converse of the test above: enforcement must not quietly restrict
    the usable range. Without this, a limiter that clamped everything to
    zero would pass the previous test.
    """
    target = legal_targets()
    code, out = send_arm_goal(target)
    assert code == 0, out
    time.sleep(1.5)

    positions = joint_positions()
    for joint, want in zip(ARM_JOINTS, target):
        got = positions[joint]
        assert abs(got - want) < 0.02, \
            f'{joint}: commanded {want:+.3f}, reached {got:+.3f}'


# -- the robot's own facts, through the client -------------------------

@pytest.mark.parametrize('scenario', ['home', 'move_to_position', 'gripper'])
def test_scenario_passes_against_mock(robot, scenario):
    """
    The scenarios that depend on a robot's own facts pass on this robot.

    `home` and `move_to_position` visit every named pose, and `gripper`
    opens and closes to the travel the profile declares. These are the
    three that read something a foreign robot has different: the poses
    were a four-element constant in the client until a five-joint arm
    stopped every scenario at home(), and the gripper width was 18 mm
    typed into a method until a gripper measured in radians needed
    opening.

    Checked here, in the cheapest tier that has a robot at all, so the
    next such assumption costs a minute to find rather than a simulator
    run. The full suite still runs against physics in `make test-sim`.
    """
    code, out = _run(
        ['ros2', 'run', 'threevn_control', 'run_scenario', scenario],
        timeout=120)
    assert code == 0, f'scenario {scenario} failed:\n{out}'
    assert 'PASS' in out, out

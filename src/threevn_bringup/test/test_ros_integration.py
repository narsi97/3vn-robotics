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

from conftest import run as _run
import pytest

pytestmark = pytest.mark.ros

STARTUP_TIMEOUT = 90.0
POLL = 2.0

EXPECTED_CONTROLLERS = {
    'joint_state_broadcaster',
    'arm_controller',
    'gripper_controller',
}

ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint',
              'elbow_joint', 'wrist_joint']


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

    The mimic joint deliberately does not - it has no ros2_control
    interface - so it is excluded here and checked through TF instead.
    """
    code, out = _run(['timeout', '6', 'ros2', 'topic', 'echo',
                      '/joint_states', '--once'], timeout=15)
    for joint in ARM_JOINTS + ['gripper_left_finger_joint']:
        assert joint in out, f'{joint} missing from /joint_states'
    assert 'gripper_right_finger_joint' not in out, \
        'the mimic joint should not be in /joint_states'


# -- limit enforcement, the Phase 2 finding, now closed ----------------

def _send_arm_goal(positions, seconds=2):
    joints = ', '.join(ARM_JOINTS)
    pos = ', '.join(str(p) for p in positions)
    goal = (f'{{trajectory: {{joint_names: [{joints}], '
            f'points: [{{positions: [{pos}], '
            f'time_from_start: {{sec: {seconds}}}}}]}}}}')
    return _run(['ros2', 'action', 'send_goal',
                 '/arm_controller/follow_joint_trajectory',
                 'control_msgs/action/FollowJointTrajectory', goal],
                timeout=60)


def _joint_position(name):
    code, out = _run(['timeout', '6', 'ros2', 'topic', 'echo',
                      '/joint_states', '--once'], timeout=15)
    lines = out.splitlines()
    names_idx = next(i for i, ln in enumerate(lines) if ln.startswith('name:'))
    pos_idx = next(i for i, ln in enumerate(lines) if ln.startswith('position:'))
    names = [ln.strip('- ').strip() for ln in lines[names_idx + 1:pos_idx]]
    rest = lines[pos_idx + 1:]
    values = []
    for ln in rest:
        stripped = ln.strip()
        if not stripped.startswith('- '):
            break
        values.append(float(stripped[2:]))
    return dict(zip(names, values))[name]


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
    """
    _send_arm_goal([0.0, 0.0, 0.0, 0.0])
    time.sleep(1.0)

    code, out = _send_arm_goal([math.pi, 0.0, 0.0, 0.0])
    assert code == 0, out
    time.sleep(1.5)

    actual = _joint_position('shoulder_pan_joint')
    limit = math.pi / 2
    assert actual <= limit + 0.01, (
        f'shoulder_pan_joint reached {actual:.5f} rad '
        f'({math.degrees(actual):.1f} deg) against a limit of {limit:.5f} rad '
        '- enforce_command_limits is not working'
    )


def test_in_limit_command_is_not_clamped(robot):
    """
    A legal command is tracked exactly.

    The converse of the test above: enforcement must not quietly restrict
    the usable range. Without this, a limiter that clamped everything to
    zero would pass the previous test.
    """
    target = [0.4, 0.3, -0.2, 0.1]
    code, out = _send_arm_goal(target)
    assert code == 0, out
    time.sleep(1.5)

    for joint, want in zip(ARM_JOINTS, target):
        got = _joint_position(joint)
        assert abs(got - want) < 0.02, \
            f'{joint}: commanded {want:+.3f}, reached {got:+.3f}'

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
Shared fixtures for the ROS integration tier.

The robot is launched ONCE per session and shared by every module.
Launching per module meant two robots overlapping while the first was
still shutting down, which the duplicate-node check correctly caught -
two robot_state_publishers on one graph is exactly the condition that
made Gazebo spawn the wrong plugin back in Phase 2.

WHICH robot is THREEVN_PROFILE, set by `make test-ros PROFILE=...`. The
tests name no joint and no frame of their own: they read ROBOT below,
which is built from the description that robot runs. So the same files
are the integration suite for the 3VN arm and for an arm nobody here
designed, and a test that cannot apply to a robot says so with a skip
and a reason rather than being written around it.
"""

import os
import signal
import subprocess
import time

import pytest
from threevn_robot_description.describe import robot_facts

#: The robot under test. One environment variable, so that the Makefile,
#: CI and a developer at a shell all choose it the same way.
PROFILE = os.environ.get('THREEVN_PROFILE', 'threevn_arm_v1')

#: Its joints, limits, frames and named poses, from its description.
#: Built at import, because pytest parametrizes at collection time. If
#: the description cannot be built -- the SO-101's has to be fetched
#: first -- collection fails here with the reason, which is the earliest
#: and clearest place for it.
ROBOT = robot_facts(PROFILE, target='mock')

#: Frames every later phase depends on. The 3VN arm promises four; a
#: foreign arm promises the tool frame its profile names.
REQUIRED_FRAMES = {
    'arm': ['tool0', 'grasp_frame', 'camera_optical_frame',
            'mount_plate_link'],
}.get(ROBOT['kind'], [ROBOT['tool_frame']])

STARTUP_TIMEOUT = 90.0
POLL = 2.0

EXPECTED_CONTROLLERS = {
    'joint_state_broadcaster',
    'arm_controller',
    'gripper_controller',
}


def pytest_configure(config):
    """Declare the custom marker."""
    config.addinivalue_line(
        'markers',
        'ros: test needs a running ROS graph (excluded from `make test`)')


def run(args, timeout=30.0, cwd=None):
    """Run a command and return (returncode, combined output)."""
    proc = subprocess.run(
        args, capture_output=True, text=True, timeout=timeout, check=False,
        cwd=cwd)
    return proc.returncode, proc.stdout + proc.stderr


def controllers_active():
    """Return True when every expected controller reports active."""
    code, out = run(['ros2', 'control', 'list_controllers'])
    if code != 0:
        return False
    active = {line.split()[0] for line in out.splitlines()
              if line.strip().endswith('active')}
    return EXPECTED_CONTROLLERS <= active


@pytest.fixture(scope='session')
def robot():
    """
    Launch the mock robot once, yield when ready, tear down at the end.

    Teardown kills the whole process group. `ros2 launch` forks the
    control node, robot_state_publisher and three spawners; killing only
    the parent leaves nodes holding the graph, so the next run sees
    duplicates and fails for reasons unrelated to the code.
    """
    # Anything left from a previous run would produce duplicate nodes.
    subprocess.run(['pkill', '-9', '-f', 'ros2_control_no[d]e'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'robot_state_pub[l]isher'], check=False)
    time.sleep(2)

    proc = subprocess.Popen(
        ['ros2', 'launch', 'threevn_bringup', 'robot.launch.py',
         'target:=mock', f'profile:={PROFILE}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=dict(os.environ), start_new_session=True)
    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(f'robot exited early, code {proc.returncode}')
            if controllers_active():
                break
            time.sleep(POLL)
        else:
            pytest.fail(f'controllers not active within {STARTUP_TIMEOUT:.0f}s')
        time.sleep(3.0)          # let TF fill
        yield proc
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=20)
        except Exception:                              # noqa: BLE001
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:                          # noqa: BLE001
                pass
        time.sleep(3)


def legal_targets(scale=1.0):
    """
    Return one in-limit, non-zero position per arm joint.

    A different fraction of each joint's travel, toward whichever limit
    is further from zero. Different on purpose: equal targets would not
    notice two joints swapped, and zero would not notice a limiter that
    clamps everything to nothing.
    """
    targets = []
    for index, joint in enumerate(ROBOT['arm_joints']):
        lower, upper = ROBOT['limits'][joint]
        fraction = scale * (0.10 + 0.04 * index)
        wider = upper if upper >= -lower else lower
        targets.append(round(fraction * wider, 4))
    return targets


def send_arm_goal(positions, seconds=2):
    """Send a one-point trajectory straight to the action, by CLI."""
    joints = ', '.join(ROBOT['arm_joints'])
    pos = ', '.join(str(p) for p in positions)
    goal = (f'{{trajectory: {{joint_names: [{joints}], '
            f'points: [{{positions: [{pos}], '
            f'time_from_start: {{sec: {seconds}}}}}]}}}}')
    return run(['ros2', 'action', 'send_goal',
                '/arm_controller/follow_joint_trajectory',
                'control_msgs/action/FollowJointTrajectory', goal],
               timeout=60)


def joint_positions():
    """Read the live joint positions from /joint_states, by CLI."""
    code, out = run(['timeout', '6', 'ros2', 'topic', 'echo',
                     '/joint_states', '--once'], timeout=15)
    lines = out.splitlines()
    names_idx = next(i for i, ln in enumerate(lines) if ln.startswith('name:'))
    pos_idx = next(i for i, ln in enumerate(lines) if ln.startswith('position:'))
    names = [ln.strip('- ').strip() for ln in lines[names_idx + 1:pos_idx]]
    values = []
    for ln in lines[pos_idx + 1:]:
        stripped = ln.strip()
        if not stripped.startswith('- '):
            break
        values.append(float(stripped[2:]))
    return dict(zip(names, values))

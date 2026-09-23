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
"""

import os
import signal
import subprocess
import time

import pytest

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


def run(args, timeout=30.0):
    """Run a command and return (returncode, combined output)."""
    proc = subprocess.run(
        args, capture_output=True, text=True, timeout=timeout, check=False)
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
        ['ros2', 'launch', 'threevn_bringup', 'robot.launch.py', 'target:=mock'],
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

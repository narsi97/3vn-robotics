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
The `slow` marker, and the simulator every slow test module drives.

The simulator is managed as a subprocess rather than through
launch_testing. That is a deliberate trade: launch_testing is the
idiomatic choice, but wrapping a process that forks Gazebo, an ROS bridge
and three controller spawners makes failures hard to attribute. A plain
subprocess with explicit polling and unconditional teardown produces
diagnosable failures, which matters more for a test that only runs on
demand and in CI.
"""

import os
import signal
import subprocess
import time

import pytest

#: The robot under test, chosen by `make test-sim PROFILE=...`.
PROFILE = os.environ.get('THREEVN_PROFILE', 'threevn_arm_v1')


def pytest_configure(config):
    config.addinivalue_line(
        'markers', 'slow: test requires a running simulator (excluded from `make test`)'
    )


#: Gazebo, the bridge and three spawners have to come up in sequence.
STARTUP_TIMEOUT = 120.0
POLL = 2.0

EXPECTED_CONTROLLERS = {
    'joint_state_broadcaster',
    'arm_controller',
    'gripper_controller',
}


def run(args, timeout=30.0):
    """Run a command and return (returncode, stdout)."""
    proc = subprocess.run(
        args, capture_output=True, text=True, timeout=timeout, check=False)
    return proc.returncode, proc.stdout


def all_controllers_active():
    """Return True when every expected controller reports active."""
    code, out = run(['ros2', 'control', 'list_controllers'])
    if code != 0:
        return False
    active = {
        line.split()[0] for line in out.splitlines()
        if line.strip().endswith('active')
    }
    return EXPECTED_CONTROLLERS <= active


# Module scope: each slow module gets a robot that has just spawned.
@pytest.fixture(scope='module')
def simulator():
    """
    Start the Gazebo stack, yield once controllers are active, tear down.

    Teardown kills the whole process group: `ros2 launch` forks Gazebo and
    several spawners, and killing only the parent leaves a Gazebo holding
    the ROS graph, which makes every subsequent test fail for reasons that
    have nothing to do with the code under test.
    """
    env = dict(os.environ)
    proc = subprocess.Popen(
        ['ros2', 'launch', 'threevn_sim', 'sim.launch.py',
         f'profile:={PROFILE}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=env, start_new_session=True)

    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(
                    f'simulator exited early with code {proc.returncode}')
            if all_controllers_active():
                break
            time.sleep(POLL)
        else:
            pytest.fail(
                f'controllers not active within {STARTUP_TIMEOUT:.0f}s; '
                f'expected {sorted(EXPECTED_CONTROLLERS)}')
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
        # gz forks its own server; make sure nothing survives to poison
        # the ROS graph for the next test run.
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'ruby.*gz'], check=False)
        time.sleep(3)

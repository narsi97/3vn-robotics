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
Gazebo integration: the robot spawns, controllers activate, scenarios pass.

Marked `slow` and excluded from `make test`; `make test-sim` runs it.
Starting a simulator takes the better part of a minute, and a suite that
slow is a suite nobody runs before committing.

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

pytestmark = pytest.mark.slow

#: Gazebo, the bridge and three spawners have to come up in sequence.
STARTUP_TIMEOUT = 120.0
POLL = 2.0

EXPECTED_CONTROLLERS = {
    'joint_state_broadcaster',
    'arm_controller',
    'gripper_controller',
}


def _run(args, timeout=30.0):
    """Run a command and return (returncode, stdout)."""
    proc = subprocess.run(
        args, capture_output=True, text=True, timeout=timeout, check=False)
    return proc.returncode, proc.stdout


def _all_controllers_active():
    """Return True when every expected controller reports active."""
    code, out = _run(['ros2', 'control', 'list_controllers'])
    if code != 0:
        return False
    active = {
        line.split()[0] for line in out.splitlines()
        if line.strip().endswith('active')
    }
    return EXPECTED_CONTROLLERS <= active


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
        ['ros2', 'launch', 'threevn_sim', 'sim.launch.py'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=env, start_new_session=True)

    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(
                    f'simulator exited early with code {proc.returncode}')
            if _all_controllers_active():
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


def test_controllers_are_active(simulator):
    """All three controllers reach the active state."""
    assert _all_controllers_active()


def test_hardware_component_is_active(simulator):
    """The ros2_control hardware component is loaded and active."""
    code, out = _run(['ros2', 'control', 'list_hardware_components'])
    assert code == 0, out
    assert 'threevn_arm' in out
    assert 'label=active' in out


def test_all_command_interfaces_are_claimed(simulator):
    """
    Every actuated joint has its command interface claimed.

    An unclaimed interface means a controller loaded but is not actually
    driving that joint, which looks like a working robot right up until
    one joint silently does nothing.
    """
    code, out = _run(['ros2', 'control', 'list_hardware_components'])
    assert code == 0, out
    for joint in ('shoulder_pan_joint', 'shoulder_lift_joint',
                  'elbow_joint', 'wrist_joint', 'gripper_left_finger_joint'):
        assert f'{joint}/position [available] [claimed]' in out, \
            f'{joint} command interface not claimed'


def test_joint_states_publish_at_roughly_the_configured_rate(simulator):
    """
    /joint_states arrives at approximately the configured update rate.

    controllers.yaml configures 50 Hz, and at idle this measures 49.8 Hz.
    The band below is wide on purpose: the observed rate is the configured
    rate multiplied by Gazebo's real-time factor, and on CPU-only physics
    (llvmpipe, no GPU) the RTF measures between 0.59 and 0.99 depending on
    what else the machine is doing. Under the load of this very suite the
    rate lands near 40 Hz.

    So this test's job is to catch a MISCONFIGURED update_rate - 10 Hz or
    200 Hz would both be bugs - not to benchmark the host. The real-time
    factor is asserted separately below, which is where a catastrophically
    slow simulator gets caught.
    """
    code, out = _run(['timeout', '8', 'ros2', 'topic', 'hz', '/joint_states'],
                     timeout=20)
    rates = [
        float(line.split()[-1])
        for line in out.splitlines() if 'average rate' in line
    ]
    assert rates, f'no rate reported:\n{out}'
    assert 25.0 <= rates[0] <= 65.0, (
        f'expected ~50 Hz scaled by real-time factor, got {rates[0]:.1f} Hz. '
        'Check controller_manager update_rate in '
        'threevn_bringup/config/controllers.yaml.'
    )


def test_real_time_factor_is_usable(simulator):
    """
    Gazebo keeps a workable fraction of real time.

    A low real-time factor does not fail loudly - it silently stretches
    every duration in the simulation, so a trajectory that "takes two
    seconds" takes three and timing-sensitive results stop meaning
    anything. Worth asserting directly rather than inferring from a topic
    rate.

    The floor is deliberately generous: this is CPU physics with no GPU,
    where 0.6-1.0 is normal. Below 0.3 the simulation is not useful and
    something is wrong with the host.
    """
    code, out = _run(
        ['timeout', '10', 'gz', 'topic', '-e',
         '-t', '/world/empty_bench/stats', '-n', '5'], timeout=25)
    factors = [
        float(line.split(':')[1])
        for line in out.splitlines() if 'real_time_factor' in line
    ]
    assert factors, f'no real_time_factor reported:\n{out}'
    best = max(factors)
    assert best >= 0.3, (
        f'real-time factor peaked at {best:.2f}; the simulation is running '
        'far slower than real time, so timing results from it are not '
        'trustworthy. Check host CPU contention and Docker VM resources.'
    )


def test_sim_clock_is_running(simulator):
    """/clock is bridged, so controllers run on simulation time."""
    code, out = _run(['timeout', '6', 'ros2', 'topic', 'echo',
                      '/clock', '--once'], timeout=15)
    assert 'sec' in out, f'no /clock message:\n{out}'


@pytest.mark.parametrize('scenario', [
    'home',
    'move_joint',
    'move_to_position',
    'gripper',
    'safety_limit',
    'pick_and_place',
])
def test_scenario_passes_in_simulation(simulator, scenario):
    """
    Each scenario passes against Gazebo physics.

    These are the same scenarios that run against mock, unchanged. A
    scenario that passed in only one of the two would mean the robot
    abstraction had become target-specific.
    """
    code, out = _run(
        ['ros2', 'run', 'threevn_control', 'run_scenario', scenario],
        timeout=180)
    assert code == 0, f'scenario {scenario} failed:\n{out}'
    assert 'PASS' in out, out

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
Readiness logic. No ROS, no robot, no HTTP.

/readyz is what a deploy gate, an alert or an operator would trust, so a
bug here does not produce a wrong number on a screen - it produces a
green light over a dead robot. Every branch is exercised with a fake
clock, which also means the stale-data tests run instantly rather than
sleeping.
"""

import pytest
from threevn_dashboard.robot_state import (
    CONTROLLER_STALE_AFTER_SECONDS,
    REQUIRED_CONTROLLERS,
    RobotState,
    STALE_AFTER_SECONDS,
)


class FakeClock:
    """A clock the test drives by hand."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        """Move time forward."""
        self.now += seconds


@pytest.fixture
def clock():
    """Provide a controllable clock."""
    return FakeClock()


@pytest.fixture
def state(clock):
    """Provide a RobotState driven by the fake clock."""
    return RobotState(clock=clock)


def _make_ready(state):
    state.update_joints(['shoulder_pan_joint'], [0.1], [0.0])
    state.update_controllers([(name, 'active') for name in REQUIRED_CONTROLLERS])


def test_starts_not_ready(state):
    """A robot nobody has heard from is not ready."""
    ready, reasons = state.readiness_detail()
    assert not ready
    assert any('no joint state' in r for r in reasons)


def test_ready_when_joints_fresh_and_controllers_active(state):
    """The happy path."""
    _make_ready(state)
    ready, reasons = state.readiness_detail()
    assert ready, reasons
    assert reasons == []


def test_not_ready_when_joint_state_goes_stale(state, clock):
    """
    Fresh controllers do not make a silent robot ready.

    This is the failure that matters: the control stack is nominally fine
    and no data is arriving. A check that only looked at controllers would
    report a healthy robot with a disconnected arm.
    """
    _make_ready(state)
    assert state.is_ready()

    clock.advance(STALE_AFTER_SECONDS + 0.1)

    ready, reasons = state.readiness_detail()
    assert not ready
    assert any('stale' in r for r in reasons), reasons


def test_stale_boundary_is_inclusive(state, clock):
    """Exactly at the threshold still counts as fresh."""
    _make_ready(state)
    clock.advance(STALE_AFTER_SECONDS)
    assert state.is_ready()


@pytest.mark.parametrize('missing', REQUIRED_CONTROLLERS)
def test_not_ready_when_a_required_controller_is_inactive(state, missing):
    """
    Flowing joint state does not make an uncommandable robot ready.

    The converse of the staleness case: data arrives, but the controller
    that moves the arm is not active, so nothing can actually be
    commanded.
    """
    state.update_joints(['shoulder_pan_joint'], [0.1])
    state.update_controllers([
        (name, 'inactive' if name == missing else 'active')
        for name in REQUIRED_CONTROLLERS
    ])
    ready, reasons = state.readiness_detail()
    assert not ready
    assert any(missing in r for r in reasons), reasons


def test_not_ready_when_controller_list_is_empty(state):
    """
    An empty controller list means not ready, never ready-by-default.

    The node clears the list when controller_manager is unreachable. If
    'no controllers' were treated as 'nothing wrong', losing the entire
    control stack would read as healthy.
    """
    state.update_joints(['shoulder_pan_joint'], [0.1])
    state.update_controllers([])
    ready, reasons = state.readiness_detail()
    assert not ready
    assert len(reasons) == len(REQUIRED_CONTROLLERS)


def test_snapshot_reports_joint_positions_and_velocities(state):
    """Positions and velocities are paired with the right joint names."""
    state.update_joints(['a', 'b'], [0.1, 0.2], [1.0, 2.0])
    joints = state.snapshot()['joints']
    assert joints['a']['position'] == pytest.approx(0.1)
    assert joints['b']['velocity'] == pytest.approx(2.0)


def test_missing_velocities_are_none_not_zero(state):
    """
    An absent velocity reads as None, never as 0.0.

    Reporting zero would be a lie that looks exactly like a stationary
    joint, which is the one thing a reader might act on.
    """
    state.update_joints(['a', 'b'], [0.1, 0.2], [1.0])
    joints = state.snapshot()['joints']
    assert joints['a']['velocity'] == pytest.approx(1.0)
    assert joints['b']['velocity'] is None


def test_message_counter_increments(state):
    """The snapshot counts how many joint messages have arrived."""
    for _ in range(3):
        state.update_joints(['a'], [0.0])
    assert state.snapshot()['joint_messages'] == 3


def test_snapshot_is_a_copy(state):
    """
    Mutating a snapshot cannot corrupt the live state.

    Handing out an internal reference would let one HTTP handler change
    what every other reader sees.
    """
    state.update_joints(['a'], [0.1])
    snap = state.snapshot()
    snap['joints']['a'] = 'tampered'
    snap['controllers'].append({'name': 'ghost', 'state': 'active'})
    assert state.snapshot()['joints']['a']['position'] == pytest.approx(0.1)
    assert state.snapshot()['controllers'] == []


def test_stale_controller_list_does_not_count_as_active(state, clock):
    """
    A controller list that has aged out is not evidence of anything.

    Regression test for a measured failure. Killing ros2_control_node left
    the dashboard reporting all three controllers "active" for a control
    stack that no longer existed, because rclpy's service_is_ready() keeps
    returning True after the server dies and the last good list was never
    replaced.

    Readiness stayed correct only because joint state went stale at the
    same instant - a coincidence, not a mechanism. Here joint state is
    kept deliberately fresh so the controller ageing is the only thing
    that can produce the right answer.
    """
    _make_ready(state)
    assert state.is_ready()

    clock.advance(CONTROLLER_STALE_AFTER_SECONDS + 0.1)
    # Joint state keeps arriving, so ONLY the controller list is stale.
    state.update_joints(['shoulder_pan_joint'], [0.1], [0.0])

    snap = state.snapshot()
    assert snap['joints_fresh'], 'joint state should still be fresh here'
    assert not snap['controllers_fresh']

    ready, reasons = state.readiness_detail()
    assert not ready, 'a stale controller list must not report ready'
    assert any('not answering' in r for r in reasons), reasons


def test_fresh_controller_list_is_marked_fresh(state):
    """The happy path still reports the list as current."""
    _make_ready(state)
    assert state.snapshot()['controllers_fresh']


def test_stale_hardware_list_is_marked_stale(state, clock):
    """
    The hardware list ages out like the controller list.

    Found by killing the control stack and watching the dashboard keep
    reporting the hardware component "active". Clearing it when
    service_is_ready() goes false does not work - rclpy keeps reporting a
    dead service as ready - so ageing is the only mechanism that holds.
    """
    _make_ready(state)
    state.update_hardware([('threevn_arm', 'active')])
    assert state.snapshot()['hardware_fresh']

    clock.advance(CONTROLLER_STALE_AFTER_SECONDS + 0.1)

    snap = state.snapshot()
    assert not snap['hardware_fresh']
    # The last known value is still reported, but flagged - the dashboard
    # shows it as stale rather than pretending it is current.
    assert snap['hardware'] == [{'name': 'threevn_arm', 'state': 'active'}]

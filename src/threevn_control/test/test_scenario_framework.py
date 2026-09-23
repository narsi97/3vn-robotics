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
Unit tests for the scenario framework. No robot, no simulator, no ROS.

The framework decides whether a scenario passed, so a bug here would make
every scenario result untrustworthy - including one that reports PASS
while the robot did nothing. Worth testing directly rather than trusting
it because the scenarios happen to go green.
"""

import pytest
from threevn_control.scenarios.base import (
    get,
    REGISTRY,
    Scenario,
    ScenarioFailed,
    ScenarioResult,
    StepResult,
)


class FakeRobot:
    """Stands in for RobotClient. The framework only ever spins it."""

    def __init__(self):
        self.calls = []

    def destroy_node(self):
        """No-op."""


def _spinless(monkeypatch):
    """Neutralise rclpy.spin_once so the framework runs without a node."""
    import sys
    import types
    fake = types.ModuleType('rclpy')
    fake.spin_once = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, 'rclpy', fake)


def test_step_records_success(monkeypatch):
    """A step whose check returns True is recorded as passed."""
    _spinless(monkeypatch)

    class S(Scenario):
        name = 'ok'

        def run(self):
            self.step('do a thing', lambda: None, lambda: True, settle=0)

    result = S(FakeRobot()).execute()
    assert result.passed
    assert len(result.steps) == 1
    assert result.steps[0].name == 'do a thing'


def test_failing_check_fails_the_scenario(monkeypatch):
    """
    A step whose post-condition is False fails the scenario.

    This is the assertion that matters most: if a false check could still
    produce a PASS, every scenario in the suite would be decorative.
    """
    _spinless(monkeypatch)

    class S(Scenario):
        name = 'bad'

        def run(self):
            self.step('claim something untrue', lambda: None,
                      lambda: False, settle=0)

    result = S(FakeRobot()).execute()
    assert not result.passed
    assert 'post-condition not met' in result.steps[0].detail


def test_exception_in_action_fails_the_scenario(monkeypatch):
    """An exception is recorded as a failure, with its type named."""
    _spinless(monkeypatch)

    class S(Scenario):
        name = 'boom'

        def run(self):
            def explode():
                raise RuntimeError('servo on fire')
            self.step('explode', explode, settle=0)

    result = S(FakeRobot()).execute()
    assert not result.passed
    assert 'RuntimeError' in result.steps[0].detail
    assert 'servo on fire' in result.steps[0].detail


def test_scenario_stops_at_the_first_failure(monkeypatch):
    """
    Later steps do not run once a step has failed.

    Continuing past a failed motion would command the arm from a position
    nobody verified, which on real hardware is how you break something.
    """
    _spinless(monkeypatch)
    reached = []

    class S(Scenario):
        name = 'halt'

        def run(self):
            self.step('fail here', lambda: None, lambda: False, settle=0)
            self.step('never runs', lambda: reached.append(1), settle=0)

    result = S(FakeRobot()).execute()
    assert not result.passed
    assert reached == [], 'execution continued past a failed step'
    assert len(result.steps) == 1


def test_step_raises_scenario_failed_when_called_directly(monkeypatch):
    """`step` raises so `run` bodies short-circuit; `execute` catches it."""
    _spinless(monkeypatch)

    class S(Scenario):
        name = 'raise'

        def run(self):
            self.step('nope', lambda: None, lambda: False, settle=0)

    with pytest.raises(ScenarioFailed):
        S(FakeRobot()).run()


def test_result_reports_every_step():
    """The report names each step and ends with an overall verdict."""
    result = ScenarioResult(name='demo', steps=[
        StepResult('first', True, seconds=1.0),
        StepResult('second', False, detail='broke', seconds=2.0),
    ])
    text = result.report()
    assert 'first' in text and 'second' in text
    assert 'FAIL' in text
    assert not result.passed
    assert result.seconds == pytest.approx(3.0)


def test_all_expected_scenarios_are_registered():
    """
    Importing the suite registers every scenario.

    A scenario that silently fails to register would simply never run, and
    `--all` would report success over a shorter list than intended.
    """
    from threevn_control.scenarios import arm  # noqa: F401
    expected = {
        'home', 'move_joint', 'move_to_position',
        'gripper', 'safety_limit', 'pick_and_place',
    }
    assert expected <= set(REGISTRY), \
        f'not registered: {sorted(expected - set(REGISTRY))}'


def test_get_rejects_an_unknown_name():
    """Looking up a nonexistent scenario raises rather than returning None."""
    from threevn_control.scenarios import arm  # noqa: F401
    with pytest.raises(KeyError):
        get('no_such_scenario')

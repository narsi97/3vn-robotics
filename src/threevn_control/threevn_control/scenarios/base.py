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
Scenario framework.

A scenario is a named sequence of steps, each of which does something to
the robot and then asserts what should be true afterwards. Scenarios are
written against RobotClient alone, so the same scenario runs unchanged
against mock, against Gazebo and, from Phase 5, against the physical arm.

They serve two purposes on purpose:

  * as tests   - `make test-sim` runs them and fails the build
  * as demos   - `ros2 run threevn_control run_scenario pick_and_place`

Writing them once for both is what stops the demo drifting away from the
thing that is actually verified.
"""

from dataclasses import dataclass, field
import time
from typing import Callable, List


class ScenarioFailed(AssertionError):
    """A scenario step did not produce the expected state."""


@dataclass
class StepResult:
    """Outcome of one step."""

    name: str
    passed: bool
    detail: str = ''
    seconds: float = 0.0


@dataclass
class ScenarioResult:
    """Outcome of a whole scenario."""

    name: str
    steps: List[StepResult] = field(default_factory=list)

    @property
    def passed(self):
        """Return True when every step passed."""
        return all(s.passed for s in self.steps)

    @property
    def seconds(self):
        """Return the total wall time across all steps."""
        return sum(s.seconds for s in self.steps)

    def report(self):
        """Return a human-readable multi-line report."""
        lines = [f'scenario: {self.name}']
        for step in self.steps:
            mark = 'ok  ' if step.passed else 'FAIL'
            lines.append(f'  {mark} {step.name:<38} {step.seconds:5.2f}s'
                         + (f'  {step.detail}' if step.detail else ''))
        verdict = 'PASS' if self.passed else 'FAIL'
        lines.append(f'  {verdict} in {self.seconds:.2f}s')
        return '\n'.join(lines)


class Scenario:
    """
    Base class for scenarios.

    Subclasses set `name` and implement `run`, calling `self.step(...)`
    for each action-then-assert pair.
    """

    name = 'unnamed'
    description = ''
    #: Scenarios needing objects in the world cannot run against mock.
    requires_simulator = False

    def __init__(self, robot):
        self.robot = robot
        self.result = ScenarioResult(name=self.name)

    def step(self, name, action, check=None, settle=0.3):
        """
        Perform one step and record whether it produced the expected state.

        `action` does the thing; `check` returns True when the world is as
        it should be afterwards. A step with no `check` is recorded as a
        step but asserts nothing - used for setup, never for the thing the
        scenario exists to prove.
        """
        started = time.monotonic()
        detail = ''
        try:
            action()
            if settle:
                deadline = time.monotonic() + settle
                import rclpy
                while time.monotonic() < deadline:
                    rclpy.spin_once(self.robot, timeout_sec=0.02)
            passed = True if check is None else bool(check())
            if not passed:
                detail = 'post-condition not met'
        except Exception as exc:                     # noqa: BLE001
            passed = False
            detail = f'{type(exc).__name__}: {exc}'

        self.result.steps.append(StepResult(
            name=name, passed=passed, detail=detail,
            seconds=time.monotonic() - started))

        if not passed:
            raise ScenarioFailed(f'{self.name} / {name}: {detail}')
        return self

    def run(self):
        """Execute the scenario. Implemented by subclasses."""
        raise NotImplementedError

    def execute(self):
        """Run the scenario and return its ScenarioResult, never raising."""
        try:
            self.run()
        except ScenarioFailed:
            pass
        return self.result


#: Populated by the registry below.
REGISTRY: dict = {}


def register(cls):
    """Class decorator adding a scenario to the registry."""
    REGISTRY[cls.name] = cls
    return cls


def get(name) -> Callable:
    """Look up a scenario class by name."""
    if name not in REGISTRY:
        raise KeyError(f'unknown scenario {name!r}; one of {sorted(REGISTRY)}')
    return REGISTRY[name]

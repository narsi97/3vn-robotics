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
The fleet reporter, with no network and no robot.

The property that matters here is NEGATIVE: nothing this does may ever
affect the robot. A dashboard that took an arm down because a VPS was
unreachable would invert the whole point of keeping control local, so
most of these tests are about failing quietly and correctly.
"""

import urllib.error

import pytest
from threevn_dashboard.fleet_reporter import FleetReporter


class FakeResponse:
    """Minimal stand-in for an HTTP response used as a context manager."""

    def __init__(self, status=202):
        self.status = status

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def make_reporter(opener, state=None, version=None, **kwargs):
    """Build a reporter with injected data and transport."""
    return FleetReporter(
        url='https://example.invalid/api/telemetry',
        token='token',
        state_fn=lambda: state if state is not None else {},
        version_fn=lambda: version if version is not None else {},
        opener=opener,
        **kwargs,
    )


# -- payload -----------------------------------------------------------

def test_the_report_carries_identity_and_health():
    """Everything the fleet view needs to answer 'is that robot alright'."""
    reporter = make_reporter(
        opener=lambda *a, **k: FakeResponse(),
        state={'ready': True, 'uptime_seconds': 12.5, 'joint_messages': 900},
        version={'robot_id': 'robot-001', 'software_version': '0.7.0',
                 'git_commit': 'abc1234', 'hardware_target': 'esp32'},
    )
    report = reporter.build_report()
    assert report['robot_id'] == 'robot-001'
    assert report['software_version'] == '0.7.0'
    assert report['hardware_target'] == 'esp32'
    assert report['ready'] is True
    assert report['joint_messages'] == 900


def test_the_report_does_not_carry_joint_positions():
    """
    Joint angles stay on the robot.

    A fleet view answers 'is it alright', not 'where is its elbow'. A
    50 Hz stream over the internet is a different system with different
    costs, and this assertion is here so that decision is made
    deliberately rather than by someone widening a dict.
    """
    reporter = make_reporter(
        opener=lambda *a, **k: FakeResponse(),
        state={'ready': True, 'joints': {'shoulder_pan_joint': {'position': 0.4}}},
    )
    assert 'joints' not in reporter.build_report()


def test_missing_data_becomes_unknown_not_a_plausible_default():
    """An absent version is reported as unknown, never as something tidy."""
    reporter = make_reporter(opener=lambda *a, **k: FakeResponse())
    report = reporter.build_report()
    assert report['robot_id'] == 'unknown'
    assert report['software_version'] == 'unknown'
    assert report['ready'] is False


def test_reasons_are_capped():
    """A pathological reason list cannot inflate every request."""
    reporter = make_reporter(
        opener=lambda *a, **k: FakeResponse(),
        state={'reasons': [f'reason {i}' for i in range(50)]},
    )
    assert len(reporter.build_report()['reasons']) <= 8


# -- failure behaviour -------------------------------------------------

def test_a_successful_send_is_counted():
    reporter = make_reporter(opener=lambda *a, **k: FakeResponse(202))
    assert reporter.send_once() is True
    assert reporter.sent == 1
    assert reporter.failed == 0


@pytest.mark.parametrize('exc', [
    urllib.error.URLError('unreachable'),
    OSError('connection refused'),
    TimeoutError('timed out'),
])
def test_network_failures_never_raise(exc):
    """
    THE test. An unreachable VPS must not propagate into the robot.

    Losing the network has to degrade observability, never motion, so
    send_once swallows everything rather than letting an exception reach
    a ROS callback.
    """
    def boom(*args, **kwargs):
        raise exc

    reporter = make_reporter(opener=boom)
    assert reporter.send_once() is False
    assert reporter.failed == 1
    assert reporter.last_error is not None


def test_an_http_error_is_a_failure_not_a_crash():
    def boom(*args, **kwargs):
        raise urllib.error.HTTPError(
            'https://example.invalid', 401, 'Unauthorized', {}, None)

    reporter = make_reporter(opener=boom)
    assert reporter.send_once() is False


def test_a_rejecting_server_counts_as_a_failure():
    reporter = make_reporter(opener=lambda *a, **k: FakeResponse(500))
    assert reporter.send_once() is False
    assert reporter.failed == 1


def test_failure_logging_is_throttled():
    """
    An unreachable VPS must not fill the robot's disk.

    One line every ten seconds is a megabyte a day of the same message.
    """
    logged = []

    class Logger:
        def warn(self, message):
            logged.append(message)

        def info(self, message):
            pass

    def boom(*args, **kwargs):
        raise OSError('unreachable')

    reporter = make_reporter(opener=boom, logger=Logger())
    for _ in range(90):
        reporter.send_once()

    assert reporter.failed == 90
    assert len(logged) <= 4, f'logged {len(logged)} times for 90 failures'


# -- configuration -----------------------------------------------------

def test_not_configured_means_no_reporter(monkeypatch):
    """A robot on a bench should not be trying to phone home."""
    monkeypatch.delenv('THREEVN_FLEET_URL', raising=False)
    assert FleetReporter.from_env(dict, dict) is None


def test_a_url_without_a_token_refuses_to_start(monkeypatch):
    """
    Starting anyway would send reports the server rejects forever.

    Better to say so once at startup than to retry into a 401 every ten
    seconds for the life of the robot.
    """
    monkeypatch.setenv('THREEVN_FLEET_URL', 'https://example.invalid/x')
    monkeypatch.delenv('THREEVN_FLEET_TOKEN', raising=False)

    warned = []

    class Logger:
        def warn(self, message):
            warned.append(message)

    assert FleetReporter.from_env(dict, dict, logger=Logger()) is None
    assert warned, 'it should say why it is not starting'


def test_fully_configured_produces_a_reporter(monkeypatch):
    monkeypatch.setenv('THREEVN_FLEET_URL', 'https://example.invalid/x')
    monkeypatch.setenv('THREEVN_FLEET_TOKEN', 'token')
    assert FleetReporter.from_env(dict, dict) is not None


def test_a_bad_interval_falls_back_rather_than_crashing(monkeypatch):
    monkeypatch.setenv('THREEVN_FLEET_URL', 'https://example.invalid/x')
    monkeypatch.setenv('THREEVN_FLEET_TOKEN', 'token')
    monkeypatch.setenv('THREEVN_FLEET_INTERVAL', 'not-a-number')
    assert FleetReporter.from_env(dict, dict) is not None


def test_stop_is_safe_when_never_started():
    make_reporter(opener=lambda *a, **k: FakeResponse()).stop()

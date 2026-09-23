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
The OTLP metrics exporter, with no collector and no robot.

Two things are being protected here.

The PROTOCOL: this hand-builds OTLP/HTTP JSON rather than using the SDK,
so the envelope shape is our responsibility. A malformed batch is
rejected silently by a collector and shows up as a dashboard that is
simply empty.

The ROBOT: nothing this does may affect it. A monitoring backend going
down must never propagate into the machine with motors.
"""

import json
import urllib.error

import pytest
from threevn_dashboard.otlp_metrics import (
    build_payload,
    counter,
    CUMULATIVE,
    gauge,
    OtlpMetricsExporter,
)


class FakeResponse:
    def __init__(self, status=200):
        self.status = status

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def capturing_opener(sink, status=200):
    """Return an opener that records the request instead of sending it."""
    def _open(request, timeout=None):
        sink.append(request)
        return FakeResponse(status)
    return _open


def make_exporter(opener, state=None, version=None, **kwargs):
    return OtlpMetricsExporter(
        endpoint='http://collector.invalid/v1/metrics',
        state_fn=lambda: state if state is not None else {},
        version_fn=lambda: version if version is not None else {},
        opener=opener,
        **kwargs,
    )


# -- envelope shape ----------------------------------------------------

def test_gauge_has_the_otlp_shape():
    point = gauge('threevn.test', 1.5, unit='s')['gauge']['dataPoints'][0]
    assert point['asDouble'] == 1.5
    assert point['timeUnixNano'].isdigit()


def test_counter_is_monotonic_and_cumulative():
    """
    Temporality matters, and getting it wrong is silent.

    DELTA would need the collector to convert; a Prometheus-style backend
    reading DELTA as CUMULATIVE shows counters that appear to reset.
    """
    total = counter('threevn.test', 42, start_ns=123)['sum']
    assert total['isMonotonic'] is True
    assert total['aggregationTemporality'] == CUMULATIVE
    assert total['dataPoints'][0]['asInt'] == '42'


def test_counter_reports_the_process_start_not_now():
    """
    Report the process start time, not the current time.

    startTimeUnixNano must be when the process began.

    A backend uses it to detect restarts. Setting it to 'now' makes a
    counter that reset look like one that ran backwards.
    """
    point = counter('threevn.test', 7, start_ns=999)['sum']['dataPoints'][0]
    assert point['startTimeUnixNano'] == '999'
    assert point['timeUnixNano'] != '999'


def test_payload_nests_resource_and_scope_correctly():
    payload = build_payload({'robot.id': 'robot-001'}, [gauge('x', 1)])
    resource = payload['resourceMetrics'][0]
    assert resource['resource']['attributes'][0]['key'] == 'robot.id'
    assert resource['scopeMetrics'][0]['metrics'][0]['name'] == 'x'


def test_attribute_types_are_chosen_correctly():
    """A string value typed as an int is rejected by the collector."""
    payload = build_payload(
        {'s': 'text', 'i': 5, 'f': 1.5, 'b': True}, [])
    kinds = {
        a['key']: next(iter(a['value']))
        for a in payload['resourceMetrics'][0]['resource']['attributes']
    }
    assert kinds == {'s': 'stringValue', 'i': 'intValue',
                     'f': 'doubleValue', 'b': 'boolValue'}


def test_the_whole_payload_is_json_serialisable():
    """It goes over the wire as JSON; anything exotic fails at send time."""
    exporter = make_exporter(opener=lambda *a, **k: FakeResponse())
    json.dumps(build_payload(
        exporter.resource_attributes(), exporter.build_metrics()))


# -- content -----------------------------------------------------------

def test_resource_attributes_identify_the_robot():
    """Without these a metric from a fleet is unattributable."""
    exporter = make_exporter(
        opener=lambda *a, **k: FakeResponse(),
        version={'robot_id': 'robot-001', 'software_version': '0.7.0',
                 'git_commit': 'abc1234', 'hardware_target': 'esp32'},
    )
    attrs = exporter.resource_attributes()
    assert attrs['robot.id'] == 'robot-001'
    assert attrs['service.version'] == '0.7.0'
    assert attrs['git.commit'] == 'abc1234'
    assert attrs['robot.hardware_target'] == 'esp32'


def test_the_expected_metrics_are_present():
    exporter = make_exporter(
        opener=lambda *a, **k: FakeResponse(),
        state={'ready': True, 'uptime_seconds': 10.0, 'joint_messages': 500,
               'joint_state_age_seconds': 0.02, 'joints_fresh': True,
               'controllers': [{'name': 'arm_controller', 'state': 'active'}],
               'hardware': [{'name': 'threevn_arm', 'state': 'active'}]},
    )
    names = {m['name'] for m in exporter.build_metrics()}
    assert {'threevn.robot.ready', 'threevn.robot.uptime',
            'threevn.controllers.active', 'threevn.hardware.active',
            'threevn.joint_states.received',
            'threevn.joint_states.age'} <= names


def test_counts_are_not_declared_as_ratios():
    """
    Unit drives the exported metric NAME, not just documentation.

    The OTLP -> Prometheus translation appends a suffix from the unit:
    unit '1' becomes _ratio, which made a count of three controllers
    arrive as `threevn_controllers_active_ratio`.
    """
    exporter = make_exporter(
        opener=lambda *a, **k: FakeResponse(),
        state={'controllers': [{'name': 'a', 'state': 'active'}]},
    )
    by_name = {m['name']: m for m in exporter.build_metrics()}
    assert by_name['threevn.controllers.active']['unit'] != '1'
    assert by_name['threevn.hardware.active']['unit'] != '1'


def test_only_active_controllers_are_counted():
    exporter = make_exporter(
        opener=lambda *a, **k: FakeResponse(),
        state={'controllers': [
            {'name': 'a', 'state': 'active'},
            {'name': 'b', 'state': 'inactive'},
            {'name': 'c', 'state': 'active'},
        ]},
    )
    by_name = {m['name']: m for m in exporter.build_metrics()}
    point = by_name['threevn.controllers.active']['gauge']['dataPoints'][0]
    assert point['asDouble'] == 2.0


def test_age_is_omitted_before_the_first_message_not_sent_as_zero():
    """
    Absence is the signal.

    Emitting 0 would read as 'perfectly fresh', which is the exact
    opposite of 'nothing has arrived yet'.
    """
    exporter = make_exporter(
        opener=lambda *a, **k: FakeResponse(),
        state={'joint_state_age_seconds': None},
    )
    names = {m['name'] for m in exporter.build_metrics()}
    assert 'threevn.joint_states.age' not in names


# -- failure behaviour -------------------------------------------------

def test_a_successful_export_is_counted():
    sent = []
    exporter = make_exporter(opener=capturing_opener(sent))
    assert exporter.export_once() is True
    assert exporter.sent == 1
    assert len(sent) == 1


def test_the_request_carries_json_and_any_auth_header():
    sent = []
    exporter = make_exporter(
        opener=capturing_opener(sent),
        headers={'Authorization': 'Bearer tok'})
    exporter.export_once()
    request = sent[0]
    assert request.get_header('Content-type') == 'application/json'
    assert request.get_header('Authorization') == 'Bearer tok'


@pytest.mark.parametrize('exc', [
    urllib.error.URLError('unreachable'),
    OSError('connection refused'),
    TimeoutError('timed out'),
])
def test_an_unreachable_collector_never_raises(exc):
    """
    THE test. Monitoring going down must not reach the robot.

    A robot that crashed because a metrics backend was unreachable would
    be a worse robot than one with no metrics at all.
    """
    def boom(*args, **kwargs):
        raise exc

    exporter = make_exporter(opener=boom)
    assert exporter.export_once() is False
    assert exporter.failed == 1


def test_export_failures_are_themselves_a_metric():
    """
    A gap in the graphs is otherwise unattributable.

    This lets a reader tell 'the robot stopped' from 'the pipeline
    stopped', which is guesswork without it.
    """
    def boom(*args, **kwargs):
        raise OSError('unreachable')

    exporter = make_exporter(opener=boom)
    exporter.export_once()
    names = {m['name'] for m in exporter.build_metrics()}
    assert 'threevn.otlp.export.failures' in names


def test_failure_logging_is_throttled():
    logged = []

    class Logger:
        def warn(self, message):
            logged.append(message)

        def info(self, message):
            pass

    def boom(*args, **kwargs):
        raise OSError('unreachable')

    exporter = make_exporter(opener=boom, logger=Logger())
    for _ in range(60):
        exporter.export_once()
    assert exporter.failed == 60
    assert len(logged) <= 4


# -- configuration -----------------------------------------------------

def test_not_configured_means_no_exporter(monkeypatch):
    monkeypatch.delenv('THREEVN_OTLP_ENDPOINT', raising=False)
    assert OtlpMetricsExporter.from_env(dict, dict) is None


def test_configured_endpoint_produces_an_exporter(monkeypatch):
    monkeypatch.setenv('THREEVN_OTLP_ENDPOINT', 'http://c/v1/metrics')
    assert OtlpMetricsExporter.from_env(dict, dict) is not None


def test_a_token_becomes_a_bearer_header(monkeypatch):
    monkeypatch.setenv('THREEVN_OTLP_ENDPOINT', 'http://c/v1/metrics')
    monkeypatch.setenv('THREEVN_OTLP_TOKEN', 'tok')
    sent = []
    exporter = OtlpMetricsExporter.from_env(dict, dict)
    exporter._opener = capturing_opener(sent)
    exporter.export_once()
    assert sent[0].get_header('Authorization') == 'Bearer tok'


def test_stop_is_safe_when_never_started():
    make_exporter(opener=lambda *a, **k: FakeResponse()).stop()

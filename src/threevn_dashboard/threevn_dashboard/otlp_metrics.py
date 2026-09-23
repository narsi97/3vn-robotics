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
Push robot metrics as OTLP over HTTP, using nothing but the standard library.

OTLP was chosen for one reason: instrument once and the BACKEND becomes
swappable. VictoriaMetrics today, something else later, by editing a
collector config rather than re-instrumenting a robot.

This speaks OTLP/HTTP with a JSON body, which is part of the
specification and which a real OpenTelemetry Collector accepts and parses
- verified before this file was written, not assumed.

Why not the OpenTelemetry SDK
  It would pull roughly 20 MB and a dozen packages onto the device. The
  ROS image has no pip at all, so it would also mean adding one. This is
  the constrained, safety-adjacent machine that will eventually sit on a
  network, and Phase 6 spent real effort getting its image down to
  1.67 GB by removing things it does not need.

  The honest cost of this choice: no traces, and no auto-instrumentation.
  Those are what the SDK is genuinely good at, and when a trace spanning
  perception -> planning -> action is actually wanted (Phase 11+), that
  is when the dependency earns itself. Metrics do not need it.

Everything here fails silently. A robot must never be affected by a
monitoring backend being down.
"""

import json
import os
import threading
import time
import urllib.error
import urllib.request

#: OTLP aggregation temporality. 2 == CUMULATIVE, which is what
#: Prometheus-style backends expect; DELTA would need the collector to
#: convert, and getting it wrong makes counters look like they reset.
CUMULATIVE = 2

DEFAULT_INTERVAL = 15.0
REQUEST_TIMEOUT = 5.0


def _attr(key, value):
    """Build one OTLP KeyValue, choosing the right value type."""
    if isinstance(value, bool):
        return {'key': key, 'value': {'boolValue': value}}
    if isinstance(value, int):
        return {'key': key, 'value': {'intValue': str(value)}}
    if isinstance(value, float):
        return {'key': key, 'value': {'doubleValue': value}}
    return {'key': key, 'value': {'stringValue': str(value)}}


def gauge(name, value, unit='1', description='', attributes=None):
    """
    Build an OTLP gauge: a value that can go up or down.

    Uptime, ages, counts of currently-active things.
    """
    return {
        'name': name,
        'unit': unit,
        'description': description,
        'gauge': {
            'dataPoints': [{
                'asDouble': float(value),
                'timeUnixNano': str(time.time_ns()),
                'attributes': [_attr(k, v) for k, v in (attributes or {}).items()],
            }],
        },
    }


def counter(name, value, start_ns, unit='1', description='', attributes=None):
    """
    Build an OTLP monotonic sum: a value that only increases.

    `start_ns` must be when the process started, not now. A backend uses
    it to detect restarts; getting it wrong makes a counter that reset
    look like one that ran backwards.
    """
    return {
        'name': name,
        'unit': unit,
        'description': description,
        'sum': {
            'aggregationTemporality': CUMULATIVE,
            'isMonotonic': True,
            'dataPoints': [{
                'asInt': str(int(value)),
                'startTimeUnixNano': str(start_ns),
                'timeUnixNano': str(time.time_ns()),
                'attributes': [_attr(k, v) for k, v in (attributes or {}).items()],
            }],
        },
    }


def build_payload(resource_attributes, metrics):
    """Wrap metrics in the OTLP envelope."""
    return {
        'resourceMetrics': [{
            'resource': {
                'attributes': [
                    _attr(k, v) for k, v in resource_attributes.items()
                ],
            },
            'scopeMetrics': [{
                'scope': {'name': 'threevn_dashboard'},
                'metrics': metrics,
            }],
        }],
    }


class OtlpMetricsExporter:
    """
    Periodically pushes robot metrics to an OTLP/HTTP endpoint.

    Push, not scrape. A robot behind home NAT cannot be scraped, and more
    importantly it must not wait on a monitoring backend.
    """

    def __init__(self, endpoint, state_fn, version_fn, headers=None,
                 interval=DEFAULT_INTERVAL, logger=None, opener=None):
        self._endpoint = endpoint
        self._state_fn = state_fn
        self._version_fn = version_fn
        self._headers = dict(headers or {})
        self._interval = interval
        self._log = logger
        self._opener = opener or urllib.request.urlopen
        self._stop = threading.Event()
        self._thread = None
        self._start_ns = time.time_ns()
        self._sent = 0
        self._failed = 0
        self._last_error = None

    @classmethod
    def from_env(cls, state_fn, version_fn, logger=None):
        """
        Build from the environment, or None when not configured.

        Unset is the normal case. A robot on a bench should not be trying
        to reach a monitoring stack.
        """
        endpoint = os.environ.get('THREEVN_OTLP_ENDPOINT', '').strip()
        if not endpoint:
            return None
        headers = {}
        token = os.environ.get('THREEVN_OTLP_TOKEN', '').strip()
        if token:
            headers['Authorization'] = f'Bearer {token}'
        try:
            interval = float(
                os.environ.get('THREEVN_OTLP_INTERVAL', DEFAULT_INTERVAL))
        except ValueError:
            interval = DEFAULT_INTERVAL
        return cls(endpoint, state_fn, version_fn, headers=headers,
                   interval=interval, logger=logger)

    # -- metric construction -------------------------------------------

    def resource_attributes(self):
        """
        Identify the robot these metrics came from.

        Without these a metric from a fleet is unattributable, which is
        most of what makes a version block worth having in the first
        place.
        """
        version = self._version_fn() or {}
        return {
            'service.name': 'threevn_robot',
            'service.version': version.get('software_version', 'unknown'),
            'robot.id': version.get('robot_id', 'unknown'),
            'robot.model': version.get('robot_model', 'unknown'),
            'robot.hardware_target': version.get('hardware_target', 'unknown'),
            'robot.firmware_version': version.get('firmware_version', 'unknown'),
            'git.commit': version.get('git_commit', 'unknown'),
        }

    def build_metrics(self):
        """Assemble the current metric set."""
        state = self._state_fn() or {}
        controllers = state.get('controllers') or []
        active = sum(1 for c in controllers if c.get('state') == 'active')
        hardware = state.get('hardware') or []
        hardware_active = sum(1 for h in hardware if h.get('state') == 'active')

        age = state.get('joint_state_age_seconds')

        metrics = [
            gauge('threevn.robot.ready', 1 if state.get('ready') else 0,
                  description='1 when the robot is usable, 0 otherwise'),
            gauge('threevn.robot.uptime', state.get('uptime_seconds', 0.0),
                  unit='s', description='dashboard uptime'),
            # Unit matters here, not just as documentation. The
            # OTLP -> Prometheus translation appends a suffix derived
            # from it: unit "1" becomes _ratio, which made a count of 3
            # controllers read as `threevn_controllers_active_ratio`.
            gauge('threevn.controllers.active', active, unit='{controller}',
                  description='controllers reporting active'),
            gauge('threevn.hardware.active', hardware_active, unit='{component}',
                  description='hardware components reporting active'),
            counter('threevn.joint_states.received', state.get('joint_messages', 0),
                    self._start_ns,
                    description='joint state messages received since start'),
        ]

        # Age is None before the first message. Emitting 0 would read as
        # "perfectly fresh", which is the opposite of the truth, so the
        # metric is omitted and the absence is the signal.
        if age is not None:
            metrics.append(
                gauge('threevn.joint_states.age', age, unit='s',
                      description='seconds since the last joint state'))

        # Freshness flags, as separate series so an alert can name the
        # specific thing that went stale rather than a composite.
        for key, name in (
            ('joints_fresh', 'threevn.joint_states.fresh'),
            ('controllers_fresh', 'threevn.controllers.fresh'),
            ('hardware_fresh', 'threevn.hardware.fresh'),
        ):
            if key in state:
                metrics.append(
                    gauge(name, 1 if state[key] else 0,
                          description=f'1 when {key.replace("_", " ")}'))

        metrics.append(
            counter('threevn.otlp.export.failures', self._failed,
                    self._start_ns,
                    description='failed metric exports; a robot reporting '
                                'this is still healthy, the pipeline is not'))
        return metrics

    # -- transport -----------------------------------------------------

    def export_once(self):
        """
        Push one batch. Returns True on success, never raises.

        A robot that crashed because a metrics backend was unreachable
        would be a worse robot than one with no metrics.
        """
        try:
            payload = build_payload(
                self.resource_attributes(), self.build_metrics())
            request = urllib.request.Request(
                self._endpoint,
                data=json.dumps(payload).encode(),
                method='POST',
                headers={'Content-Type': 'application/json', **self._headers},
            )
            with self._opener(request, timeout=REQUEST_TIMEOUT) as response:
                ok = 200 <= getattr(response, 'status', 200) < 300
        except (urllib.error.URLError, urllib.error.HTTPError,
                OSError, ValueError, TypeError) as exc:
            self._failed += 1
            self._last_error = f'{type(exc).__name__}: {exc}'
            # Throttled. An unreachable collector must not fill the
            # robot's disk with one line every fifteen seconds.
            if self._log and self._failed % 20 == 1:
                self._log.warn(
                    f'OTLP export failed ({self._failed} so far): '
                    f'{self._last_error}')
            return False

        if ok:
            self._sent += 1
            return True
        self._failed += 1
        self._last_error = 'collector rejected the batch'
        return False

    def start(self):
        """Begin exporting on a daemon thread."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name='threevn-otlp-exporter', daemon=True)
        self._thread.start()
        if self._log:
            self._log.info(
                f'exporting metrics to {self._endpoint} '
                f'every {self._interval:.0f}s')

    def stop(self):
        """Stop exporting."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self):
        while not self._stop.is_set():
            self.export_once()
            self._stop.wait(self._interval)

    @property
    def sent(self):
        """Return the number of successful exports."""
        return self._sent

    @property
    def failed(self):
        """Return the number of failed exports."""
        return self._failed

    @property
    def last_error(self):
        """Return the most recent failure, or None."""
        return self._last_error

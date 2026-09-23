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
Push a status summary to the fleet view on the VPS.

The robot reports OUT. Nothing reaches in.

Two reasons, and the second is the important one. Robots sit behind home
NAT where inbound connections do not work. And more fundamentally, the
robot must never wait on the VPS: losing the network has to degrade
observability, never motion. So every failure here is swallowed after a
throttled log line, and the reporter runs on its own thread where it
cannot block a ROS callback.

Opt-in. With no THREEVN_FLEET_URL set, nothing starts and nothing is
sent - a robot on a bench should not be trying to phone home.

What crosses the wire is deliberately a SUBSET of what the local
dashboard knows: health, versions, readiness. Joint angles at 50 Hz have
no business on the internet, and a fleet view needs to answer "is that
robot alright" rather than "where is its elbow".
"""

import json
import os
import threading
import urllib.error
import urllib.request

#: Every 10s. The server treats 35s of silence as an outage, so this
#: tolerates two dropped requests before anyone draws conclusions.
DEFAULT_INTERVAL = 10.0

#: A robot must not sit waiting on a slow server.
REQUEST_TIMEOUT = 5.0


class FleetReporter:
    """
    Periodically POSTs a status summary, on its own thread.

    Constructed with callables rather than a node, so it can be tested
    with no ROS, no robot and no network.
    """

    def __init__(self, url, token, state_fn, version_fn,
                 interval=DEFAULT_INTERVAL, logger=None, opener=None):
        self._url = url
        self._token = token
        self._state_fn = state_fn
        self._version_fn = version_fn
        self._interval = interval
        self._log = logger
        # Injectable so tests never touch the network.
        self._opener = opener or urllib.request.urlopen
        self._stop = threading.Event()
        self._thread = None
        self._sent = 0
        self._failed = 0
        self._last_error = None

    @classmethod
    def from_env(cls, state_fn, version_fn, logger=None):
        """
        Build a reporter from the environment, or None if not configured.

        Absence is the normal case: most robots are on a bench.
        """
        url = os.environ.get('THREEVN_FLEET_URL', '').strip()
        if not url:
            return None
        token = os.environ.get('THREEVN_FLEET_TOKEN', '').strip()
        if not token:
            if logger:
                logger.warn(
                    'THREEVN_FLEET_URL is set but THREEVN_FLEET_TOKEN is not; '
                    'the server will reject every report, so not starting')
            return None
        try:
            interval = float(os.environ.get('THREEVN_FLEET_INTERVAL', DEFAULT_INTERVAL))
        except ValueError:
            interval = DEFAULT_INTERVAL
        return cls(url, token, state_fn, version_fn,
                   interval=interval, logger=logger)

    # -- lifecycle -----------------------------------------------------

    def start(self):
        """Begin reporting on a daemon thread."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name='threevn-fleet-reporter', daemon=True)
        self._thread.start()
        if self._log:
            self._log.info(
                f'reporting to {self._url} every {self._interval:.0f}s')

    def stop(self):
        """Stop reporting and wait briefly for the thread."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- payload -------------------------------------------------------

    def build_report(self):
        """
        Assemble what gets sent.

        A subset by design. If this ever grows joint positions, the
        reason to stop is that a fleet view answers "is it alright", not
        "where is its elbow" - and a 50 Hz stream over the internet is a
        different system with different costs.
        """
        state = self._state_fn() or {}
        version = self._version_fn() or {}
        return {
            'robot_id': version.get('robot_id', 'unknown'),
            'software_version': version.get('software_version', 'unknown'),
            'firmware_version': version.get('firmware_version', 'unknown'),
            'git_commit': version.get('git_commit', 'unknown'),
            'robot_model': version.get('robot_model', 'unknown'),
            'hardware_target': version.get('hardware_target', 'unknown'),
            'ready': bool(state.get('ready', False)),
            'uptime_seconds': float(state.get('uptime_seconds', 0.0)),
            'joint_messages': int(state.get('joint_messages', 0)),
            'reasons': list(state.get('reasons', []))[:8],
        }

    # -- internals -----------------------------------------------------

    def send_once(self):
        """
        Send one report. Returns True on success.

        Never raises. A robot that crashed because a dashboard was
        unreachable would be a worse robot.
        """
        try:
            payload = json.dumps(self.build_report()).encode()
            request = urllib.request.Request(
                self._url, data=payload, method='POST',
                headers={
                    'Content-Type': 'application/json',
                    'Authorization': f'Bearer {self._token}',
                })
            with self._opener(request, timeout=REQUEST_TIMEOUT) as response:
                ok = 200 <= getattr(response, 'status', 200) < 300
        except (urllib.error.URLError, urllib.error.HTTPError,
                OSError, ValueError, TypeError) as exc:
            self._failed += 1
            self._last_error = f'{type(exc).__name__}: {exc}'
            # Throttled: an unreachable VPS must not fill the robot's
            # disk with one log line every ten seconds.
            if self._log and self._failed % 30 == 1:
                self._log.warn(
                    f'fleet report failed ({self._failed} so far): '
                    f'{self._last_error}')
            return False

        if ok:
            self._sent += 1
            return True
        self._failed += 1
        self._last_error = 'server rejected the report'
        return False

    def _run(self):
        while not self._stop.is_set():
            self.send_once()
            # Waiting on the event rather than sleeping means stop() is
            # immediate instead of taking up to a full interval.
            self._stop.wait(self._interval)

    # -- introspection -------------------------------------------------

    @property
    def sent(self):
        """Return the number of successful reports."""
        return self._sent

    @property
    def failed(self):
        """Return the number of failed attempts."""
        return self._failed

    @property
    def last_error(self):
        """Return the most recent failure, or None."""
        return self._last_error

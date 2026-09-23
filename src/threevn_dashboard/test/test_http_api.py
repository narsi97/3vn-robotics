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
The HTTP surface, against a real server on a real socket. No ROS.

These are the contracts something else depends on - a monitor polling
/readyz, a deploy gate reading its status code, a browser opening the
stream - so they are exercised end to end rather than by calling handler
methods directly.
"""

import json
import socket
import urllib.error
import urllib.request

import pytest
from threevn_dashboard.http_server import make_server, serve_in_background
from threevn_dashboard.robot_state import REQUIRED_CONTROLLERS, RobotState
from threevn_dashboard.version import version_info


def _free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


@pytest.fixture
def state():
    """Provide a RobotState the test can drive."""
    return RobotState()


@pytest.fixture
def base_url(state):
    """Run a real server on a free port for the duration of a test."""
    port = _free_port()
    server = make_server(
        state=state,
        version_info=version_info,
        readiness_detail=state.readiness_detail,
        host='127.0.0.1',
        port=port,
    )
    serve_in_background(server)
    try:
        yield f'http://127.0.0.1:{port}'
    finally:
        server.shutdown()
        server.server_close()


def _get(url, timeout=5):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.status, response.read(), dict(response.headers)


def _get_json(url):
    status, body, headers = _get(url)
    return status, json.loads(body), headers


def make_ready(state):
    """Put the state into a ready condition."""
    state.update_joints(['shoulder_pan_joint'], [0.25], [0.0])
    state.update_controllers([(n, 'active') for n in REQUIRED_CONTROLLERS])


def test_healthz_is_liveness_only(base_url, state):
    """
    /healthz answers OK even when the robot is unusable.

    Liveness asks 'is this process able to respond'. Conflating it with
    readiness would make a container orchestrator restart a perfectly
    healthy dashboard every time the arm was switched off.
    """
    status, body, _ = _get_json(f'{base_url}/healthz')
    assert status == 200
    assert body['status'] == 'ok'
    assert body['check'] == 'liveness'
    assert not state.is_ready()


def test_readyz_returns_503_when_not_ready(base_url):
    """
    /readyz answers 503 with reasons when the robot is unusable.

    The status code carries the signal so a monitor need not parse the
    body to act.
    """
    with pytest.raises(urllib.error.HTTPError) as caught:
        _get(f'{base_url}/readyz')
    assert caught.value.code == 503
    body = json.loads(caught.value.read())
    assert body['ready'] is False
    assert body['reasons'], 'a 503 must explain itself'


def test_readyz_returns_200_when_ready(base_url, state):
    """/readyz answers 200 with no reasons once the robot is usable."""
    make_ready(state)
    status, body, _ = _get_json(f'{base_url}/readyz')
    assert status == 200
    assert body['ready'] is True
    assert body['reasons'] == []


def test_state_reports_live_joint_positions(base_url, state):
    """/api/state reflects what the robot last reported."""
    make_ready(state)
    status, body, _ = _get_json(f'{base_url}/api/state')
    assert status == 200
    assert body['joints']['shoulder_pan_joint']['position'] == pytest.approx(0.25)
    assert body['ready'] is True


def test_version_exposes_every_required_field(base_url):
    """
    The version block carries everything a failure report needs.

    Without these a fault cannot be tied back to a commit, a profile or a
    deployment, which is most of what makes a physical system debuggable.
    """
    status, body, _ = _get_json(f'{base_url}/api/version')
    assert status == 200
    for field in ('robot_id', 'software_version', 'git_commit',
                  'firmware_version', 'robot_model', 'config_profile',
                  'hardware_target', 'deployed_at', 'reported_at'):
        assert field in body, f'version block is missing {field}'


def test_api_responses_are_not_cacheable(base_url):
    """
    State must never be cached.

    A dashboard showing a cached snapshot of a moving robot is worse than
    one showing nothing, because it looks correct.
    """
    for path in ('/api/state', '/api/version', '/healthz'):
        _, _, headers = _get(f'{base_url}{path}')
        assert headers.get('Cache-Control') == 'no-store', path


def test_dashboard_page_is_served(base_url):
    """/ returns the dashboard HTML."""
    status, body, headers = _get(base_url + '/')
    assert status == 200
    assert 'text/html' in headers.get('Content-Type', '')
    assert b'3VN' in body


def test_unknown_path_is_404(base_url):
    """An unrecognised route returns 404, not a stack trace."""
    with pytest.raises(urllib.error.HTTPError) as caught:
        _get(f'{base_url}/nope')
    assert caught.value.code == 404


def test_static_handler_refuses_path_traversal(base_url):
    """
    A request cannot escape the static directory.

    This server binds 0.0.0.0 inside a container and will eventually sit
    behind a public reverse proxy, so ../ handling is not academic.
    """
    for attack in ('/../version.py', '/..%2fversion.py', '/../../etc/passwd'):
        try:
            status, body, _ = _get(f'{base_url}{attack}')
        except urllib.error.HTTPError as err:
            assert err.code in (400, 404), attack
        else:
            assert b'SOFTWARE_VERSION' not in body, f'{attack} leaked a file'
            assert b'root:' not in body, f'{attack} leaked a file'


def test_stream_emits_server_sent_events(base_url, state):
    """
    /api/stream delivers SSE frames a browser can consume.

    Checked on the wire rather than by inspecting the handler: the
    Content-Type and the `data: ` framing are the actual contract with
    EventSource.
    """
    make_ready(state)
    request = urllib.request.Request(f'{base_url}/api/stream')
    with urllib.request.urlopen(request, timeout=5) as response:
        assert response.headers['Content-Type'] == 'text/event-stream'
        assert response.headers.get('Cache-Control') == 'no-store'
        first = response.readline()
        assert first.startswith(b'data: '), first
        payload = json.loads(first[len(b'data: '):])
        assert payload['ready'] is True
        assert 'joints' in payload

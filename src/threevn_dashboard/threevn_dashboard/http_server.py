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
The HTTP surface, built on the standard library.

No web framework. aiohttp, Flask and FastAPI are all absent from the ROS
image, and adding one to serve five endpoints and a static page would be
a dependency, a licence entry and an attack surface bought for nothing.

Server-Sent Events rather than a WebSocket, deliberately:

  * A dashboard only needs one direction. Commands are ordinary POSTs.
  * SSE is plain HTTP, so it traverses the shared Caddy with no special
    configuration - no Upgrade handling, no per-route exception.
  * It satisfies the estate-wide `connect-src 'self'` CSP when served
    under the same origin, so shipping this does NOT require loosening a
    policy that every other 3VN product shares.

That last point is why the CSP problem flagged in Phase 1 mostly
disappears instead of needing a Report-Only rollout.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import pathlib
import threading
import time

STATIC_DIR = pathlib.Path(__file__).parent / 'static'

#: Interval between SSE frames. 5 Hz is far below the 50 Hz control loop
#: and still well past what a human eye resolves in a browser; pushing
#: every control cycle would burn CPU redrawing text nobody can read.
STREAM_INTERVAL = 0.2

#: Keeps proxies from closing an idle stream.
KEEPALIVE_INTERVAL = 15.0


class DashboardHandler(BaseHTTPRequestHandler):
    """Routes for the dashboard. One instance per request."""

    #: Injected by make_server.
    state = None
    version_info = None
    readiness_detail = None

    protocol_version = 'HTTP/1.1'
    server_version = 'threevn-dashboard'

    def log_message(self, fmt, *args):
        """Silence per-request logging; ROS logging carries what matters."""

    # -- helpers -------------------------------------------------------

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, indent=2).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        # A dashboard that shows a cached snapshot is actively misleading.
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, body, content_type, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_static(self, name):
        path = (STATIC_DIR / name).resolve()
        # Refuse anything that escapes the static directory.
        if not str(path).startswith(str(STATIC_DIR.resolve())) or not path.is_file():
            self._send_json({'error': 'not found'}, status=404)
            return
        ctype = mimetypes.guess_type(str(path))[0] or 'application/octet-stream'
        self._send_bytes(path.read_bytes(), ctype)

    # -- routes --------------------------------------------------------

    def do_GET(self):  # noqa: N802  (BaseHTTPRequestHandler's naming)
        """Dispatch a GET request."""
        route = self.path.split('?', 1)[0].rstrip('/') or '/'

        if route == '/':
            self._send_static('index.html')

        elif route == '/healthz':
            # LIVENESS. This process is running and can answer. It says
            # nothing about the robot -- see /readyz.
            self._send_json({'status': 'ok', 'check': 'liveness'})

        elif route == '/readyz':
            # READINESS. Is the ROBOT usable? Returns 503 when not, so a
            # load balancer, a deploy gate or a monitor can act on it
            # without parsing the body.
            ready, reasons = self.readiness_detail()
            self._send_json(
                {'ready': ready, 'check': 'readiness', 'reasons': reasons},
                status=200 if ready else 503)

        elif route == '/api/version':
            self._send_json(self.version_info())

        elif route == '/api/state':
            self._send_json(self.state.snapshot())

        elif route == '/api/stream':
            self._stream()

        else:
            self._send_json({'error': 'not found', 'path': route}, status=404)

    def _stream(self):
        """Stream state snapshots as Server-Sent Events until disconnect."""
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'keep-alive')
        # Defeats proxy buffering, which otherwise holds events until the
        # buffer fills and makes a live stream look frozen.
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()

        last_keepalive = time.monotonic()
        try:
            while True:
                payload = json.dumps(self.state.snapshot())
                self.wfile.write(f'data: {payload}\n\n'.encode())
                self.wfile.flush()

                now = time.monotonic()
                if now - last_keepalive >= KEEPALIVE_INTERVAL:
                    self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
                    last_keepalive = now

                time.sleep(STREAM_INTERVAL)
        except (BrokenPipeError, ConnectionResetError):
            # The browser navigated away. Entirely normal.
            pass


def make_server(state, version_info, readiness_detail, host='0.0.0.0', port=8107):
    """
    Build the HTTP server. Does not start it.

    Threading, because an SSE connection occupies its handler for as long
    as the browser tab stays open. A single-threaded server would answer
    one dashboard and then hang for everyone else, including the health
    probe.
    """
    handler = type('BoundDashboardHandler', (DashboardHandler,), {
        'state': state,
        'version_info': staticmethod(version_info),
        'readiness_detail': staticmethod(readiness_detail),
    })
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    return server


def serve_in_background(server):
    """Run the server on a daemon thread and return the thread."""
    thread = threading.Thread(
        target=server.serve_forever, name='threevn-dashboard-http', daemon=True)
    thread.start()
    return thread

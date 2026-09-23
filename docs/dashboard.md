# Dashboard and health

```bash
make sim &        # or: make mock &
make dash         # -> http://localhost:8107/
```

Live joint positions, controller and hardware state, readiness with
reasons, and the version block — served by a ROS node that runs **on the
robot**, not on a remote server. A dashboard that needs the internet to
tell you the arm is fine is a dashboard that lies during an outage.

## Endpoints

| Path | Purpose |
|---|---|
| `/` | The dashboard page |
| `/healthz` | **Liveness** — is this process able to answer? |
| `/readyz` | **Readiness** — is the *robot* usable? 503 when not |
| `/api/state` | Current snapshot |
| `/api/stream` | Server-Sent Events, ~5 Hz |
| `/api/version` | The deployment version block |

## Liveness and readiness are not the same question

This is the load-bearing distinction, and it is not bureaucracy.

- **`/healthz`** asks whether this process can respond. It answers 200
  even when the arm has been unplugged for an hour.
- **`/readyz`** asks whether the robot is usable: joint state arriving
  *and* the controllers that move it active. It answers **503 with
  reasons** otherwise.

Conflating them breaks in both directions. A container orchestrator
watching a combined check would restart a perfectly healthy dashboard
every time you switched the arm off. A monitor watching only liveness
would stay green through a total robot outage.

`/readyz` is what a deploy gate, an alert or an operator should watch.

## Everything ages out

Two bugs were found by killing a running robot and watching the screen,
and both had the same shape: **the dashboard kept reporting the last
thing it knew as though it were current.**

1. After `ros2_control_node` died, all three controllers still showed
   `active`.
2. After that was fixed, the hardware component still showed `active`.

The cause is worth remembering: **`rclpy`'s `service_is_ready()` keeps
returning `True` after the server is gone.** So the poll keeps firing,
the call never completes, and the previous answer survives indefinitely.
Clearing the list when the service "goes away" does not work, because it
never appears to.

So controller and hardware data carry timestamps and age out after
`CONTROLLER_STALE_AFTER_SECONDS` (6 s, three poll cycles). Stale values
are still shown — the last known state is useful — but flagged as stale
and never counted as active.

Readiness had *appeared* correct through the first bug only because joint
state went stale at the same instant. Relying on that would have been
relying on a coincidence.

## Why Server-Sent Events, not a WebSocket

- A dashboard only needs one direction. Commands are ordinary POSTs.
- SSE is plain HTTP, so it crosses the shared Caddy with no `Upgrade`
  handling and no per-route exception.
- It satisfies the estate-wide `connect-src 'self'` CSP when served under
  the same origin.

That last point matters: the CSP blocker flagged back in Phase 1 mostly
**disappears** rather than needing a `Content-Security-Policy-Report-Only`
rollout across every 3VN product. See [deployment.md](deployment.md).

No web framework either. aiohttp, Flask and FastAPI are all absent from
the ROS image, and adding one to serve five endpoints and a static page
buys a dependency, a licence entry and an attack surface for nothing.

## The version block

```json
{
  "robot_id": "...", "software_version": "0.3.0",
  "git_commit": "974bc9d", "firmware_version": "unknown",
  "robot_model": "threevn_arm_v1", "config_profile": "threevn_arm_v1",
  "hardware_target": "mock", "deployed_at": "unknown"
}
```

Values come from the environment, injected at build or deploy time — not
read from a local git checkout, because a deployed robot has no checkout
and a value that silently falls back to whatever is on disk is worse than
one that honestly says `unknown`.

Unknown fields are shown in amber on the dashboard and logged as a warning
at startup. A robot that cannot say which commit it is running is a robot
nobody can debug, and the time to notice is before an incident.

`firmware_version` stays `unknown` until Phase 5, which is accurate.

## House-style deviation

Every other 3VN product uses a Go backend. This one is Python, because
ROS 2 has no official Go client library and the node has to subscribe to
ROS topics and call ROS services directly. Recorded here per the
convention that deviations are documented with a reason.

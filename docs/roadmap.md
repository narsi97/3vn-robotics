# Roadmap

One system that evolves. Not twelve disconnected tutorials.

## Where we are

| Phase | What | Status |
|---|---|---|
| 0 | Architecture, repository, dev environment | **done** |
| 1 | Robot description (URDF/Xacro, YAML-driven) | **done** |
| 2 | Gazebo simulation + scenarios | next |
| 3 | ROS 2 control runtime, dashboard | |
| 4 | Full automated test layers | |
| 5 | ESP32 firmware + `Esp32SystemInterface` | |
| 6 | Docker runtime images | |
| 7 | CI/CD pipelines | |
| 8 | VPS deployment | |
| 9 | Monitoring (OpenTelemetry → self-hosted stack) | |
| 10 | Mobile base | |
| 11 | Mobile manipulator | |
| 12 | Camera + perception | |
| 13 | Data pipeline | |
| 14 | Machine learning | |
| 15 | MLOps | |

## Packages that do not exist yet

Deliberately not scaffolded. An empty package is a liability: it has to be
built, linted and maintained while providing nothing. Each arrives in the
phase that first needs it.

| Package | Phase | For |
|---|---|---|
| `threevn_interfaces` | 2 | custom msgs/srvs/actions — none needed yet; `control_msgs` and `sensor_msgs` cover Phase 1 |
| `threevn_control` | 2 | higher-level motion helpers |
| `threevn_hardware` | 5 | `Esp32SystemInterface` — the plugin name is already fixed in the description |
| `threevn_dashboard` | 3 | web UI |
| `threevn_telemetry` | 6 | OpenTelemetry emission |
| `threevn_perception` | 12 | camera pipeline |
| `threevn_navigation` | 11 | Nav2 integration |
| `threevn_data` | 13 | dataset collection |
| `threevn_ml` | 14 | training, inference |
| `threevn_mlops` | 15 | registry, deployment, monitoring |
| `firmware/esp32/` | 5 | ESP32 firmware |

## Tracked debt

**Every physical value is `provenance: estimated`.** Link lengths, masses
and joint limits in `config/*.yaml` are engineering guesses. After the
first arm is fabricated, measure each link with calipers and a scale,
update the YAML, and change `provenance` to `measured`. The plausibility
tests (total mass 0.2–3.0 kg) will catch an order-of-magnitude error but
not a 20% one.

**Gazebo spawn verification is a skipped test.**
`threevn_sim/test/test_gz_spawn.py` is marked `slow` and skips. It becomes
real in Phase 2, where the simulation itself is the deliverable.

**CSP will block the Phase 3 dashboard.** The shared 3VN Caddy sets
`connect-src 'self'` for every product on the domain, so a dashboard
connecting to a `rosbridge`/`foxglove_bridge` WebSocket on another origin
will be blocked. Either proxy the WebSocket under the same origin, or add
a targeted `connect-src` — shipped as `Content-Security-Policy-Report-Only`
first, per that Caddyfile's own documented practice. See
[`deployment.md`](deployment.md).

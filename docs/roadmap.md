# Roadmap

One system that evolves. Not twelve disconnected tutorials.

## Where we are

| Phase | What | Status |
|---|---|---|
| 0 | Architecture, repository, dev environment | **done** |
| 1 | Robot description (URDF/Xacro, YAML-driven) | **done** |
| 2 | Gazebo simulation + scenarios | **done** |
| 3 | ROS 2 control runtime, dashboard | **done** |
| 4 | Full automated test layers | **done** |
| 5 | ESP32 firmware + `Esp32SystemInterface` | next |
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
| `threevn_hardware` | 5 | `Esp32SystemInterface` — the plugin name is already fixed in the description |
| `threevn_telemetry` | 6 | OpenTelemetry emission |
| `threevn_perception` | 12 | camera pipeline |
| `threevn_navigation` | 11 | Nav2 integration |
| `threevn_data` | 13 | dataset collection |
| `threevn_ml` | 14 | training, inference |
| `threevn_mlops` | 15 | registry, deployment, monitoring |
| `firmware/esp32/` | 5 | ESP32 firmware |

## Mechanical design: not started

The repository has **no printable CAD**. The URDF is a kinematic model —
right lengths, masses and axes — which is what simulation, TF and planning
need. It says nothing about servo mounting pockets, 25T horn splines,
bearing seats, screw bosses, wall thickness, clearance fits or print
orientation.

The intended approach is **parametric CAD generated from the same YAML**
(CadQuery/build123d or OpenSCAD), so `threevn_arm_v1.yaml` drives both the
simulation and the printed part and the two cannot drift. Deferred until
the simulation stack is proven, per the simulation-first principle.

One design decision must be made first, because it determines the parts
and redistributes the masses:

- **Direct drive** — a servo at each joint. Simpler CAD, but servo mass
  sits out on the arm. At 55 g, `upper_arm_link`'s current mass is about
  *one MG996R* with nothing left over for structure, so today's numbers
  only hold if the servo is elsewhere.
- **Parallel linkage** — servos at the base driving distal joints through
  rods. Much better mass distribution and the genuinely clever part of the
  EEZYbot design, but harder CAD and it changes the URDF (the linkage is a
  closed kinematic chain, which URDF cannot express directly).

## Tracked debt

**Every physical value is `provenance: estimated`.** Link lengths, masses
and joint limits in `config/*.yaml` are engineering guesses. After the
first arm is fabricated, measure each link with calipers and a scale,
update the YAML, and change `provenance` to `measured`. The plausibility
tests (total mass 0.2–3.0 kg) will catch an order-of-magnitude error but
not a 20% one.

**Joint limits are now servo-derived, masses are not.** `effort` and
`velocity` come from the MG996R and SG90 datasheets at 4.8 V, and
`test_servo_limits.py` fails if any joint is configured beyond what its
servo can deliver. Link *masses* remain `provenance: estimated` and still
need measuring.

**`rclpy` reports dead services as ready.** `service_is_ready()` keeps
returning `True` after a service server has gone, so a poll keeps firing,
never completes, and the caller silently keeps its last answer. Found
twice in Phase 3 — controllers and then hardware both reported `active`
for a control stack that had been killed. Mitigated by ageing the data
out; worth remembering anywhere else a service is polled.

**~~`ros2_control` does not enforce the joint limits it declares.~~**
Closed in Phase 4 with `enforce_command_limits: true`. Kept here for the
record:
Measured in Phase 2: commanding `shoulder_pan_joint` to 180° against a 90°
limit drives it to 180° under `mock_components`, despite both the URDF
`<limit>` and the `command_interface` min/max saying otherwise. Gazebo only
clamps because its physics joint has a hard stop.

Now enforced in `ResourceManager`, below the hardware seam, and asserted
by `test_ros_integration.py`. Firmware enforcement in Phase 5 remains
non-negotiable regardless — it is the only layer that survives a host
crash. See [safety.md](safety.md).

**CSP will block the Phase 3 dashboard.** The shared 3VN Caddy sets
`connect-src 'self'` for every product on the domain, so a dashboard
connecting to a `rosbridge`/`foxglove_bridge` WebSocket on another origin
will be blocked. Either proxy the WebSocket under the same origin, or add
a targeted `connect-src` — shipped as `Content-Security-Policy-Report-Only`
first, per that Caddyfile's own documented practice. See
[`deployment.md`](deployment.md).

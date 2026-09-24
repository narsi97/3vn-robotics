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
| 5 | ESP32 firmware + `Esp32SystemInterface` | **done** (software) |
| 6 | Docker runtime images | **done** |
| 7 | CI/CD pipelines | **done** |
| 8 | VPS deployment | **done** |
| 9 | Monitoring (OpenTelemetry → self-hosted stack) | **done** |
| 10 | Mobile base | **done** (simulation) |
| 11 | Mobile manipulator | **done** (simulation); heading solved, Nav2 needs a range sensor |
| 12 | Camera + perception | **done** (simulation) |
| 13 | Data pipeline | **done** (simulation) |
| 14 | Machine learning | **done** (simulation) |
| 15 | MLOps | **done** (simulation) |

## Packages that do not exist yet

Deliberately not scaffolded. An empty package is a liability: it has to be
built, linted and maintained while providing nothing. Each arrives in the
phase that first needs it.

| Package | Phase | For |
|---|---|---|
| `threevn_interfaces` | 2 | custom msgs/srvs/actions — none needed yet; `control_msgs` and `sensor_msgs` cover Phase 1 |
| `threevn_telemetry` | 6 | OpenTelemetry emission |
| `threevn_navigation` | 11 | **exists**: gyro/wheel heading fusion. Nav2 itself is blocked on a range sensor the BOM does not have - see docs/navigation.md |

## Mechanical design: started, and deliberately undecided

The repository now has **parametric CAD for every structural part** —
base, turret, shoulder bracket, both beams, wrist and gripper —
generated from `threevn_arm_v1.yaml` by `cad/`, for BOTH candidate
mechanisms. See [cad.md](cad.md).

The ASSEMBLY parts now exist too: horn adapters, the linkage push rod,
pivot bushings and a thrust washer. The 25T spline is deliberately NOT
printed - 1.9 extrusions per tooth - so the adapter bolts to the metal
horn each servo ships with.

Every part now prints WITHOUT SUPPORT, enforced by a test, and
`make cad` emits a generated PRINTING.md with orientations, quantities
and a fastener count derived from the holes.

Enforcing printability changed the ROBOT: the pan servo is 42.9 mm tall
and the base was 30 mm, so base_link is now 45 mm and shoulder_pan's
origin moved with it. The simulation never noticed, because a URDF box
does not have to hold anything.

Still missing: an optional real thrust bearing. Nothing has been
printed.

The decision now has its number: direct drive lifts 154 g at the
shoulder, a linkage lifts 99 g, and the linkage costs 96 g more PLA.

The generated geometry also showed the URDF's link masses are wrong:
`upper_arm_link` is declared 55 g and the printed beam computes to
16.7 g. One number cannot serve both mechanisms - direct drive adds the
elbow servo to that link (71.7 g), a linkage leaves it at the base
(~20 g) - so the mass model DEPENDS on the decision below.

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

## Phase 5 is done in software, not on hardware

The firmware, the protocol and the hardware interface exist and are
tested — 53 C++ tests, none of which need a device. `make robot` now
loads the plugin and fails only because no ESP32 is attached.

What remains needs physical parts:

- **No arm has been built.** Every dimension is still
  `provenance: estimated`.
- **Docker Desktop on macOS cannot forward USB to a container**, so
  bring-up needs a Linux host, a VM, or the unimplemented `tcp`
  transport. See [firmware.md](firmware.md).
- **No hardware e-stop.** The firmware's latching stop depends on the
  firmware running correctly. A switch in the servo power line does not.
- **No position feedback.** Hobby servos report nothing, so the device
  reports its target as `measured` and a stalled servo looks healthy.

## Version drift, closed

A running robot reported `software_version: 0.3.0` while all six package
manifests said `0.1.0`. Nothing broke, which is the problem — the version
block exists to be trusted, and nobody would have noticed it was lying
until they tried to reproduce a fault from it.

`VERSION` is now the single source, and `test_version_consistency.py`
fails the build on disagreement. Firmware and protocol versions stay
deliberately independent: a host updated a dozen times can still face
firmware nobody reflashed, and one number would hide that. See
[versioning.md](versioning.md).

## Course structure: built at the end, tagged as we go

Each phase boundary is tagged (`phase-01-end` … ), so the state at the
end of any phase is recoverable. Tags are immutable and cost nothing to
maintain.

**Per-phase branches are deliberately NOT being kept.** Later phases
invalidated earlier ones: Phase 4 rewrote kinematics from YAML-driven to
URDF-driven, Phase 6 restructured the Dockerfile into six stages, Phase 7
changed `VERSION` across eight files. A branch frozen at Phase 2 would
contain a Dockerfile that contradicts the Phase 6 documentation, and
keeping fifteen branches coherent through refactors like those means
backporting every change fifteen times. That is how course repositories
rot.

The teaching history is also a **different artifact** from this
development history. Of 16 commits, four are fixes to mistakes made
earlier in the same session. A student starting a chapter wants the
previous chapter's clean end state, not `Phase 4` followed by
`repair docs mangled by an unquoted heredoc`.

Several of those mistakes are, however, the most valuable material in
the course — the CC BY-NC licence discovery, finding that nothing
enforced joint limits, a Grafana dashboard that was empty because metric
names are translated. Those belong in the narrative, not as broken
commits a student inherits.

So the course structure gets built deliberately once the platform is
complete, when the right chapter boundaries are actually known. Phases 6
and 7 are both CI/CD and will probably merge; Phase 5 may split.

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

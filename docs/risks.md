# Risks

What is known to be fragile, and what is done about it.

| Risk | Mitigation | Status |
|---|---|---|
| **Docker VM memory too low** → Gazebo OOM-killed, reads as a Gazebo bug | `make doctor` fails with the exact Settings path | **active** — the dev machine is currently at 4 GB, needs 6 GB |
| **Disk** — image ~5–6 GB plus volumes, on a host that was 85% full | Never install `ros-jazzy-desktop-full`; `make nuke`; budget stated in docker.md | monitored |
| **llvmpipe GUI at 5–15 FPS** reads as "broken" | `make sim` headless by default; `GUI=1` warns; documented as an expectation | mitigated |
| **`${target == 'x'}` NameError footgun** in xacro | Defensive `${'$(arg target)' == 'x'}` form + a test over all three targets | mitigated |
| **`--` inside an XML comment** is illegal and fails with an unhelpful message | Documented in development.md; every xacro in the repo is clean | mitigated |
| **OSRF apt repo leaking in** → incompatible `libgz-*.so`, segfault not an apt error | CI step asserts zero `gz-*` packages from osrfoundation | mitigated |
| **`xacro.load_yaml` removed upstream** would break the whole config design | CI asserts it exists on every run | mitigated |
| **Stale YAML after adding a file** — `--symlink-install` only links files present at build time | Documented; `make test` depends on `build`; tests read YAML from the same share dir xacro used | mitigated |
| **Gripper mimic joint** is not enforced in Gazebo | `mock_components` and `robot_state_publisher` honour URDF `<mimic>`; gz-sim's DART engine does not, and says so at spawn. Fallback: two independently driven fingers on one controller | **confirmed** (2026-11-18) — in physics the right finger is passive and only the left one pushes; see manipulation.md |
| **A joint parked on its hard stop** in Gazebo is held by the stop, not driven by its servo | The 3VN gripper's travel and command range sit 1 mm inside its stops; fast, ROS and Gazebo tests hold it there | **mitigated** for the 3VN gripper (2026-11-18); the SO-101 jaw keeps its raw limits as its command range, see so101.md |
| **Every physical value is a guess** | `provenance: estimated` required by test; plausibility bounds (total mass 0.2–3.0 kg); re-measure tracked in roadmap | **open by design** |
| **Scope creep across 15 phases** | Deferred-package list in roadmap.md; only 3 packages exist | mitigated |
| **CSP blocks the Phase 3 dashboard WebSocket** | Flagged in deployment.md with two concrete options | **open, Phase 3** |
| **`ros2 topic pub` stamps `sec=0`** so TF ignores it — looks like broken TF | Documented in ros2.md | mitigated |
| **VPS headroom** — monitoring needs ~650 MB, shared box has ~300–500 MB | Separate 2 vCPU / 4 GB box, ~€5.50/mo | planned, Phase 9 |

## The honest one

**Nothing here has touched a physical robot.** Every joint limit, link
length and mass is an engineer's estimate. The tests prove the
description is *internally consistent* and *physically plausible* — not
that it matches any object in the world.

That gap closes in Phase 5 and not before, and the `provenance` field
exists so it stays visible in the data rather than fading into an
assumption.

# 3VN Robotics

A software-engineering-first robotics platform. One robot system that
evolves — from a simulated 4-DOF arm, through CI, containers, deployment
and observability, to a physical mobile manipulator with perception and
machine learning.

The premise: an experienced software engineer already knows Git, Docker,
testing, CI/CD, Linux and observability. Robotics is not a separate
discipline to learn from scratch — it is a domain those skills transfer
into. This repository is built to make that transfer concrete.

**You do not need a robot to start.** Simulation-first is a hard
requirement, not a convenience: most of the course runs entirely in
Gazebo, and physical hardware is simply another execution target.

---

## Quick start

```bash
git clone https://github.com/narsi97/3vn-robotics.git
cd 3vn-robotics
make doctor      # checks Docker, RAM, disk, ports -- with exact fixes
make setup       # builds the image and the ROS workspace
make test        # the description test suite (target: under 60s)
make view        # RViz + joint sliders -> http://localhost:8106/vnc.html
```

Everything runs in a container. You need **Docker and nothing else** — no
ROS, no colcon, no Gazebo, no Python on your machine.

> Docker Desktop needs **≥ 6 GB RAM** (Settings → Resources). Below that
> Gazebo gets OOM-killed in a way that looks like a Gazebo bug.
> `make doctor` checks it.

Run `make` on its own for the full command list.

---

## The architecture, in one idea

The application does not know whether it is driving a simulator or a
physical robot.

```
                 3VN ROBOT APPLICATION
                          │
                        ROS 2
                          │
                  ros2_control
                          │
         ┌────────────────┼────────────────┐
      mock               gz              esp32
   (no sim,           Gazebo          real servos
   no hardware)      Harmonic         (Phase 5)
```

That split is a **launch argument**, not a code fork:

```bash
make mock     # no simulator, no hardware
make sim      # Gazebo Harmonic
make robot    # physical arm (Phase 5)
```

All three run the same launch files, the same controllers and the same
public API:

| ROS primitive | Name |
|---|---|
| topic | `/joint_states` |
| service | `/controller_manager/switch_controller` |
| action | `/arm_controller/follow_joint_trajectory` |
| action | `/gripper_controller/gripper_cmd` |

This is enforced, not merely documented.
[`test_ros2_control_targets.py`](src/threevn_robot_description/test/test_ros2_control_targets.py)
asserts that a `mock` or `esp32` build contains no Gazebo reference at
all, and
[`test_package_dependencies.py`](src/threevn_bringup/test/test_package_dependencies.py)
asserts that `threevn_bringup` never declares a Gazebo dependency. If
either fails, the decoupling has been broken.

---

## Repository layout

| Path | What |
|---|---|
| `src/threevn_robot_description` | URDF/Xacro + the YAML that drives it |
| `src/threevn_bringup` | Launch + controllers. **Never depends on Gazebo.** |
| `src/threevn_sim` | Gazebo. Depends on bringup, not the reverse. |
| `docker/` | The one image everything runs in |
| `docs/decisions/` | ADRs — why things are the way they are |
| `scripts/` | `doctor.sh`, description checks |

Only three ROS packages exist. Packages for later phases are listed in
[`docs/roadmap.md`](docs/roadmap.md) and deliberately **not** scaffolded —
an empty package is a liability, not a head start.

### Why `threevn_` and not `3vn_`

[REP-144](https://ros.org/reps/rep-0144.html): a ROS package name must
start with an alphabetic character, because it becomes a C++ namespace, a
Python module and a CMake target. The same rule binds topic and namespace
tokens, so `3vn_arm_v1` would parse fine as a config string and then fail
inside DDS. The repository, images and brand keep the digit;
[ADR-0001](docs/decisions/0001-package-naming.md) has the detail.

---

## One YAML holds every physical number

No dimension, mass or joint limit is hardcoded in a xacro. It all lives in
[`config/threevn_arm_v1.yaml`](src/threevn_robot_description/config/threevn_arm_v1.yaml),
read directly by xacro via `xacro.load_yaml()`.

```bash
make urdf PROFILE=threevn_arm_v1_long_reach
```

`threevn_arm_v1_long_reach.yaml` exists to prove it: same xacro, different
numbers, no code change. See
[ADR-0005](docs/decisions/0005-yaml-single-source.md).

Every mass carries a `provenance:` field — `measured`, `datasheet` or
`estimated`. Today they are all `estimated`, because no physical arm
exists yet. A test enforces the field, so that stays visible in the data
rather than hidden in a comment.

---

## Hardware

3VN Arm v1 uses **our own geometry** — primitive boxes and cylinders —
rather than an existing 3D-printable design.

**To be clear about what that is:** it is a *kinematic* model — correct
link lengths, masses, joint axes and collision volumes, enough for
physics, TF and planning. It is **not** printable CAD. There are no STLs,
no STEP files, and nothing yet describing a servo pocket, a horn spline
or a bearing seat. Designing the physical parts is tracked in
[`docs/roadmap.md`](docs/roadmap.md).

This was a licence decision before it was a technical one. The EEZYbotARM
family, the obvious starting point and the design originally specified for
this project, is **CC BY-NC 4.0** — NonCommercial — which is incompatible
with a paid course. It turned out to be the better engineering call too:
primitives are analytic in the physics engine, keep collision checking
real-time on a CPU-only container, and keep binary blobs out of Git.

If you own an EEZYbotARM, printing one for your own learning is
non-commercial use and entirely within its licence — measure it and write
a profile YAML. We link to it; we ship nothing from it.

Full audit: [`THIRD_PARTY.md`](THIRD_PARTY.md) ·
[ADR-0004](docs/decisions/0004-own-geometry.md) · BOM:
[`docs/bom-hardware.md`](docs/bom-hardware.md)

---

## Status

**Phases 0–2 complete.**

- The robot description expands for all three hardware targets.
- The arm spawns in Gazebo Harmonic, with all three controllers active
  and `/joint_states` at the configured 50 Hz.
- Six scenarios — home, move_joint, move_to_position, gripper,
  safety_limit, pick_and_place — pass against **both** Gazebo and `mock`.
- **166 fast tests in ~7 s**, plus **12 Gazebo integration tests** in
  ~106 s.

```bash
make sim &                        # Gazebo, headless
make scenario NAME=pick_and_place
make test-sim                     # the full Gazebo suite
```

Phase 3 is the control runtime and dashboard.

See [`docs/roadmap.md`](docs/roadmap.md) for the 15-phase plan and
[`docs/risks.md`](docs/risks.md) for what is known to be fragile.

## Licence

Apache-2.0. See [`LICENSE`](LICENSE) and [`THIRD_PARTY.md`](THIRD_PARTY.md).

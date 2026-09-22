# Simulation strategy

> Phase 1 ships the world, the launch file and a description that targets
> Gazebo. Scenarios and the spawn-verification tests are **Phase 2**,
> where simulation is the deliverable. This document is the strategy.

## Why simulation-first is a requirement, not a convenience

Most of this course must be completable without owning a robot. That has
an architectural consequence: hardware cannot be a special case bolted on
later, it has to be *one of several equal execution targets* from the
start. Hence the `ros2_control` seam — see
[architecture.md](architecture.md).

It also means simulation must be good enough to run in CI, which means
deterministic and headless.

## Headless, always, by default

`gz sim -s` runs the physics server with no GUI. With
`--headless-rendering` (OGRE2 + EGL) it can even render camera sensors
with **no GPU and no X server**, which is what makes both an M-series Mac
and a CI runner viable hosts.

`make sim` is headless. `GUI=1` is opt-in and warns, because on llvmpipe
the GUI runs at 5–15 FPS — a property of CPU rendering, not a fault.

## Determinism

`empty_bench.sdf` is deliberately minimal: a ground plane, one light,
nothing else. Every extra object is another contact to jitter and another
reason for a CI test to fail for reasons unrelated to the robot.

Physics is a fixed 1 ms step at real-time factor 1.0. The arm's links are
small and light; a larger step makes gripper contacts unstable.

## `/clock` is the one non-optional bridge

`config/gz_bridge.yaml` bridges `/clock` and nothing else.

Without it, controllers run on wall time while the simulator runs on
simulation time, and every trajectory completes at the wrong moment for
reasons that are very hard to see.

Joint states and commands are deliberately **not** bridged:
`gz_ros2_control` publishes them through `ros2_control` directly, so
bridging them would create a second, conflicting source of the same data.

## Where worlds live

`src/threevn_sim/worlds/`, not a top-level `simulation/`. Gazebo resolves
worlds via `GZ_SIM_RESOURCE_PATH`, which an ament environment hook points
at the package share directory. A directory outside any package would need
a hand-maintained absolute path in every launch file and would break the
moment the workspace moved.

## Planned scenarios (Phase 2)

`scenario_home`, `scenario_move_joint`, `scenario_move_to_position`,
`scenario_gripper`, `scenario_pick_and_place`, `scenario_safety_limit`.

Each becomes a `launch_testing` test: start the simulator, spawn, command,
assert observed state, shut down cleanly. The template is
`threevn_robot_description/test/test_rsp_publishes_tf.py`.

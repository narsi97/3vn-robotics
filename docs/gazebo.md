# Gazebo notes

Gazebo **Harmonic** (`gz sim` 8.x), paired with ROS 2 Jazzy through
`ros_gz`. Verify with `gz sim --versions`.

## Things that look like bugs and are not

**Fixed-joint links disappear.** Gazebo's SDF converter merges links
joined by fixed joints, so `tool0`, `grasp_frame` and
`camera_optical_frame` may not exist as Gazebo entities.

This is **irrelevant here**, because TF comes from
`robot_state_publisher`, not from Gazebo. Do not "fix" it by making those
joints revolute with zero range — that adds degrees of freedom to the
physics solver for no benefit.

**`base_footprint` is dropped.** It has no `<inertial>`, and the converter
discards inertial-less links. Expected: it is a pure reference frame.

**The GUI is slow.** 5–15 FPS on llvmpipe. Physics is unaffected.

**`gz sim` segfaults.** Almost always two incompatible sets of
`libgz-*.so` from mixing `packages.ros.org` with
`packages.osrfoundation.org`. See [docker.md](docker.md).

## `gz_ros2_control` hosts controller_manager

In simulation, `controller_manager` runs **inside the Gazebo process**,
started by the `gz_ros2_control-system` plugin. Two consequences:

1. `threevn_sim` needs its own top-level launch file; it cannot simply
   reuse `threevn_bringup/robot.launch.py`, which starts a standalone
   `ros2_control_node`.
2. Controllers cannot be spawned until the model exists in the simulator.
   `sim.launch.py` chains spawners off the spawn process's exit event for
   exactly this reason.

Everything below that difference — the controllers, the topics, the
actions — is identical to the `mock` and `esp32` targets.

## Spawning from the substitution, not the topic

`sim.launch.py` passes the description to `ros_gz_sim create` with
`-string`, using the **same launch substitution** that feeds
`robot_state_publisher`.

Spawning with `-topic robot_description` looks tidier and is a trap. A
stale `robot_state_publisher` left over from `make mock` also publishes
`/robot_description`, and the spawner will happily take that one. The
symptom is a pluginlib error half a minute later complaining that
`mock_components/GenericSystem` is not a Gazebo plugin — which says
nothing about the actual cause.

Passing the content directly makes simulator and TF byte-identical by
construction rather than by coincidence. `make sim` also runs `make stop`
first, which removes the stale-node problem at its source.

## Mimic joints are in TF, not in /joint_states

`gripper_right_finger_joint` follows the left finger through a URDF
`<mimic>` tag and has no `ros2_control` interface by design — only one
finger is commanded. It therefore **never appears in `/joint_states`**.

`robot_state_publisher` still resolves it and publishes the transform, so
read it from TF:

```bash
ros2 run tf2_ros tf2_echo gripper_base_link gripper_right_finger_link
```

Looking in `/joint_states` reports a perfectly working gripper as broken,
which is exactly what the gripper scenario did on first writing.

**TF is not physics, though.** gz-sim's DART engine does not support
mimic constraints, and says so once at spawn:

```
[Err] [Physics.cc:1906] Attempting to create a mimic constraint for joint
[gripper_right_finger_joint] but the chosen physics engine does not support
mimic constraints, so no constraint will be created.
```

So in the simulator the right finger is passive: RViz shows it following,
and it pushes nothing. See [manipulation.md](manipulation.md).

## A joint on its hard stop is not driven

DART gives each joint ONE constraint per step, in the order position
limit, velocity limit, servo (`JointConstraint::update()` in DART
6.13.2). A joint at or past its stop gets the stop, and its servo, which
is how `gz_ros2_control` drives a position command, is skipped. Two ways
in:

- **Sitting on the stop.** The 3VN gripper's travel used to equal its
  limits, so it spawned on its lower stop and was commanded onto both.
  Rounding left it at −5.4e‑21 m once, and from there every command
  stalled.
- **Being pushed onto it.** A force big enough to take the joint to its
  velocity limit in one step, mass × velocity limit ÷ time step, makes
  the velocity-limit constraint win instead of the servo. For a 2 g
  finger with a 0.05 m/s limit that is 0.1 N: measured, 0.05 N held and
  0.1 N ran the finger from 9 mm into its stop.

The rule that follows: a commanded position, a named pose or a spawn
position is never ON a stop. See [manipulation.md](manipulation.md) and
[so101.md](so101.md), where gravity on the jaw did the pushing.

## `<gazebo>` tags

Ignored by URDF parsers, which is what lets simulation-specific content
share one description. This project still emits them only for
`target:=gz`, so a build destined for real hardware contains no simulator
content at all — an architectural property, asserted in
`test_ros2_control_targets.py`.

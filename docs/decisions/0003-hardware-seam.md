# ADR-0003: The hardware abstraction seam is a `ros2_control` plugin

**Status:** accepted · **Date:** 2026-09-22

## Context

The project's central non-negotiable: *the application must not be
coupled to the physical ESP32*. The same ROS 2 application must drive
Gazebo or real servos, chosen as an execution target rather than a code
fork. A second requirement says not to build a proprietary RPC layer when
ROS 2 already provides the communication model.

These are the same requirement viewed twice, and `ros2_control`'s
`hardware_interface` plugin system is the idiomatic mechanism that
satisfies both.

## Decision

**`ros2_control` is mandatory, not "where appropriate."** The seam is a
single `hardware_interface::SystemInterface` pluginlib plugin, selected
by one xacro argument:

| `target` | Plugin | Status |
|---|---|---|
| `mock` | `mock_components/GenericSystem` | off the shelf, works today |
| `gz` | `gz_ros2_control/GazeboSimSystem` | off the shelf, Phase 2 |
| `esp32` | `threevn_hardware/Esp32SystemInterface` | Phase 5, ours |

Two of the three ship with ROS, so **Phase 1 proves the seam with zero
custom hardware code**.

### What the seam is not

Recorded so Phase 5 cannot drift back toward them:

- **Not** a node that talks serial and republishes topics — that is the
  proprietary RPC layer the spec forbids.
- **Not** `#ifdef`, or a runtime branch in application code.
- **Not** two URDFs.
- **Not** a parallel `esp32_arm_controller` package duplicating the
  controller logic.

### The ESP32 transport, decided now

`Esp32SystemInterface` runs the control loop **on the host** at 50–100 Hz
and treats the ESP32 as a dumb servo driver behind a small framed binary
protocol over USB CDC or TCP.

**micro-ROS is explicitly off the control path.** There are verified
reports of a ~1–2 Hz publish ceiling for micro-ROS on ESP32, which is two
orders of magnitude below a usable servo control rate. Because the seam is
a plugin, the transport is entirely private to `read()`/`write()` and can
change later without touching anything above it. micro-ROS becomes an
optional advanced module, not a dependency.

## Consequences

The public API is fixed on day one and is identical for every target:

| ROS primitive | Name | Why this primitive |
|---|---|---|
| topic | `/joint_states` | continuous state, many subscribers, lossy is fine |
| service | `/controller_manager/switch_controller` | short, synchronous, needs a reply |
| action | `/arm_controller/follow_joint_trajectory` | long-running, needs feedback and cancellation |
| action | `/gripper_controller/gripper_cmd` | same |

That is exactly the topics/services/actions split the specification asks
for, obtained for free rather than designed by hand.

**The acceptance test for this ADR:** a byte-identical
`ros2 action send_goal` succeeds against `mock` and against `gz`. Two
tests enforce it continuously — `test_ros2_control_targets.py` asserts a
`mock` or `esp32` expansion contains no Gazebo reference at all, and
`test_package_dependencies.py` asserts `threevn_bringup` never declares a
Gazebo dependency.

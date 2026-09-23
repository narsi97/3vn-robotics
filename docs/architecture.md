# Architecture

## The one idea

The application does not know whether it is driving a simulator or a
physical robot.

```
                    3VN ROBOT APPLICATION
                              │
                            ROS 2
                              │
                   ros2_control / controller_manager
                              │
                    hardware_interface plugin
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
   mock_components     gz_ros2_control      threevn_hardware
   GenericSystem       GazeboSimSystem      Esp32SystemInterface
          │                   │                   │
     no sim, no HW      Gazebo Harmonic      real servos (Phase 5)
```

Selecting between them is a **launch argument**:

```bash
make mock     # target:=mock
make sim      # target:=gz
make robot    # target:=esp32
```

Everything above the plugin — kinematics, controllers, topics, actions,
tests, the future dashboard — is byte-identical in all three cases.

## Why `ros2_control` and not a bridge node

The obvious alternative is a node that talks serial to the ESP32 and
republishes ROS topics. That is the "proprietary RPC layer" the project
specification forbids, and it fails the decoupling requirement: the
application would be talking to *that node's* interface, which differs
from the simulator's.

`hardware_interface::SystemInterface` is the idiomatic ROS 2 answer. Two
of the three implementations ship with ROS, so **Phase 1 proves the seam
with zero custom hardware code** — which is why the architecture is
demonstrable before any servo exists.

## The ROS interface, and why each primitive

Fixed on day one, identical across targets, obtained free from
`ros2_control` rather than designed by hand:

| Primitive | Name | Why |
|---|---|---|
| topic | `/joint_states` | Continuous state. Many subscribers, no reply, dropping a sample is harmless. |
| service | `/controller_manager/switch_controller` | Short, synchronous, needs a success/failure answer. |
| action | `/arm_controller/follow_joint_trajectory` | Long-running, needs progress feedback and cancellation. |
| action | `/gripper_controller/gripper_cmd` | Same. |

A topic cannot report failure; a service blocks and cannot be cancelled.
Motion is the only one of these that takes seconds and can be interrupted,
so it is the only one that is an action.

## Package boundaries are load-bearing

```
threevn_robot_description    URDF/Xacro + config. Depends on nothing.
          ▲
threevn_bringup              Launch + controllers.
                             MUST NOT depend on Gazebo.
          ▲
threevn_sim                  Gazebo. Depends on bringup.
```

The direction matters. `threevn_bringup` is what runs on the real robot;
if it ever gained a Gazebo dependency, `rosdep install` on the robot would
pull in the whole simulator and the decoupling claim would be quietly
false.

A comment cannot prevent that.
`threevn_bringup/test/test_package_dependencies.py` parses `package.xml`
and fails if any dependency matches Gazebo. The boundary is enforced by
CI, not by discipline.

## One description, four consumers

| Consumer | How it gets the description |
|---|---|
| **TF2** | `robot_state_publisher` consumes `robot_description`, publishes `/tf_static` (fixed joints) and `/tf` (from `/joint_states`). |
| **RViz** | RobotModel display subscribes to `/robot_description`. No separate model file. |
| **Gazebo** | `ros_gz_sim create -topic robot_description` spawns the identical string. |
| **Tests** | `xacro.process_file()` on the same entry point with the same arguments. |

**`robot_state_publisher` is the only source of TF in this repository.**
No hand-written `static_transform_publisher`, ever.

A consequence worth knowing so nobody "fixes" it: Gazebo's SDF converter
merges fixed-joint links, so `tool0` and `camera_optical_frame` may not
exist as Gazebo links. That is irrelevant here precisely because TF comes
from RSP, not from Gazebo.

Gazebo-specific content lives in `<gazebo>` tags, which URDF parsers
ignore — which is exactly what lets one file serve all four consumers
safely. Those tags are still emitted only for `target:=gz`, because a
hardware build should contain no simulator content at all.

## Configuration is the robot

Every physical number lives in `config/*.yaml`, read directly by xacro via
`xacro.load_yaml()`. The only xacro argument is the file path, so a
different robot is a different YAML file and no code change.
`threevn_arm_v1_long_reach.yaml` exists to prove exactly that.

See [ADR-0005](decisions/0005-yaml-single-source.md).

## What this leaves room for

- **Phase 5, ESP32.** A new package implementing one plugin class. Nothing
  above the seam changes.
- **Phase 11, mobile manipulator.** `base_footprint` and `mount_plate_link`
  already exist, so attaching a base is one joint.
- **Phase 12, perception.** `camera_optical_frame` already carries the
  REP-103 rotation.
- **Multiple robots.** Every macro takes a `prefix`, and a test asserts it
  reaches every link.


## Where things run, after Phase 8



**Nothing reaches into the robot.** Robots sit behind home NAT where
inbound connections do not work — but the real reason is that a robot
must never wait on the VPS. Losing the network degrades observability,
never motion.

Demonstrated rather than asserted: killing the fleet service leaves the
robot at  200 with scenarios passing, and the reporter emits a
single throttled warning.

The fleet view shows a robot as **stale** after 35 s and refuses to call
it ready. A robot that stopped reporting is not healthy just because its
last message said so — that converts an outage into a silent one.

# ROS 2 for people who already write software

You know services, message queues and RPC. ROS 2 gives you three
communication primitives, and picking the wrong one is the most common
design mistake.

| Primitive | Analogy | Use when | Cannot |
|---|---|---|---|
| **Topic** | pub/sub bus | continuous state, many consumers | report failure |
| **Service** | synchronous RPC | short request needing an answer | be cancelled; long calls block |
| **Action** | long-running job with progress + cancel | motion, anything taking seconds | be cheap |

This project's interface, and the reasoning:

| Name | Primitive | Why |
|---|---|---|
| `/joint_states` | topic | Continuous, many subscribers, dropping a sample is harmless. |
| `/controller_manager/switch_controller` | service | Short, needs a success/failure answer. |
| `/arm_controller/follow_joint_trajectory` | action | Takes seconds, needs feedback, must be cancellable. |
| `/gripper_controller/gripper_cmd` | action | Same. |

Motion is the only thing here that takes time and can be interrupted, so
it is the only action. That is the whole heuristic.

## Things with no direct analogy

**TF2** is a time-indexed transform graph. Ask "where was the gripper
relative to the camera 0.3 s ago" and it interpolates. In this repo,
`robot_state_publisher` is the *only* thing that writes to it — never a
hand-rolled `static_transform_publisher`.

**Parameters** are per-node configuration, set at launch. They are not a
config file and not a database; anything that must be shared belongs in a
YAML both nodes read.

**Lifecycle nodes** have explicit configure/activate transitions, so a
node can hold resources before being asked to do work. `ros2_control`
controllers are the example here.

**DDS discovery** finds nodes automatically over the network. Convenient
until two developers on the same LAN silently join each other's graph.
This project sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Jazzy's
replacement for the deprecated `ROS_LOCALHOST_ONLY`) to prevent that.

## Useful commands

```bash
ros2 node list
ros2 topic list                       # add -t for types
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 action list
ros2 param list /robot_state_publisher

ros2 control list_hardware_components   # expect 1, state: active
ros2 control list_controllers           # expect 3, all active

ros2 run tf2_ros tf2_echo base_link tool0
ros2 run tf2_tools view_frames          # writes frames.pdf
```

## A gotcha that will cost you an hour

`ros2 topic pub` stamps messages with `sec=0`. TF treats a zero timestamp
as ancient, so `tf2_echo` finds nothing and it looks like TF is broken. To
drive joints by hand, publish from a small node that stamps with
`self.get_clock().now().to_msg()`.

Relatedly: `pkill -f joint_state_publisher` matches **its own command
line** and kills the shell running it. Use `pkill -f 'joint_state_pub[l]isher'`.

# Safety

> **This is an educational hobby robot, not industrial equipment.**
> MG996R servos have enough torque to hurt, and metal gears do not slip.

## A measured finding: nothing below the seam enforces joint limits

This is not a hypothetical. It was measured in Phase 2.

The URDF declares `<limit lower="-1.5708" upper="1.5708">` on
`shoulder_pan_joint`, **and** the `ros2_control` block declares
`<param name="min">` / `<param name="max">` on its command interface.
Commanding that joint to 180° produced:

| Target | Result |
|---|---|
| `mock_components/GenericSystem` | **joint reported at +3.14159 rad (180°)** — limit ignored entirely |
| `gz_ros2_control/GazeboSimSystem` | clamped at +1.5708 rad (90°) |

Gazebo clamps because its *physics joint* has a hard stop. That is a
property of the simulator, not of our software. Nothing in
`ros2_control`'s default path enforced the declared limits.

Two conclusions follow, and both shaped the design:

1. **A limit check that only passes under Gazebo is testing the
   simulator.** The `safety_limit` scenario originally did exactly that:
   green in simulation, red on mock, against identical robot software.
2. **Validation has to live above the hardware seam**, where it applies
   to mock, Gazebo and the ESP32 alike.

So `RobotClient` now parses joint limits from the `/robot_description`
the robot is *actually running* and raises `LimitViolation` before any
goal is sent. `safety_limit` asserts the rejection, and passes on both
targets for the same reason.

## What that is, and what it is not

It is **command validation** and defence in depth. It catches programmer
error and out-of-range goals at the point where the intent is expressed.

It is **not a safety system**:

- It only guards commands sent *through* `RobotClient`. Anything can
  publish to the action server directly.
- It cannot stop a servo that has failed, a brownout, a gear that has
  stripped, or a mechanical collision.
- It is software, and software has bugs — including, evidently, the kind
  where a declared limit is silently ignored for months.

**Never assume software limits make a physical robot safe.**

## Layers, in order of trustworthiness

| Layer | Status | Notes |
|---|---|---|
| Physical hard stops | **not designed yet** | The most trustworthy limit is one the mechanism cannot exceed. Should be part of the CAD. |
| Servo travel | inherent | An MG996R cannot exceed ~180° of its own range. |
| Firmware clamp | **Phase 5** | The ESP32 must clamp every commanded angle independently of ROS. Non-negotiable: it is the last line that survives a host crash. |
| `RobotClient` validation | **done** | Rejects out-of-limit goals before sending. |
| `ros2_control` limits | **open** | Declared and not enforced. See roadmap. |
| Physics (simulation only) | inherent | Useful in Gazebo, meaningless on hardware. |

## Practical rules

- **Cut power at the supply, not in software**, when something goes wrong.
  Have the plug or switch within reach before you power on.
- Keep fingers clear of the gripper and the elbow while powered.
- Power servos from a separate 5 V supply with a common ground — never
  from the ESP32's USB rail. Four MG996R servos can draw several amps
  under stall and will brown out the controller, which reads as random
  firmware crashes.
- Use a current-limited bench supply for first power-on if you have one.
- Do not leave the arm powered and unattended.
- Expect the first motion after any firmware change to be wrong.

## Emergency stop

Not yet implemented. Phase 5, and it needs to work at two levels:

1. A ROS-level stop that cancels goals and holds position.
2. A **hardware** cut that does not depend on the host, the network, or
   any software being responsive.

Only the second is an actual e-stop. The first is a convenience.

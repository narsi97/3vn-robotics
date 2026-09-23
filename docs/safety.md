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

Two things were done about it, and both matter.

**Phase 3 — validation above the seam.** `RobotClient` parses joint
limits from the `/robot_description` the robot is *actually running* and
raises `LimitViolation` before any goal is sent.

**Phase 4 — enforcement below the seam.** Setting
`enforce_command_limits: true` on `controller_manager` makes
`ResourceManager` clamp every command against the URDF limits. Measured:
commanding `shoulder_pan_joint` to 180° now stops it at exactly
1.57080 rad under `mock_components`, where before it went to 3.14159.

They are not redundant. The client **tells you** — it rejects loudly, so
a programmer error surfaces at the point the intent was expressed. The
enforcement **prevents it** — silently, for any caller, including one
that bypasses the client entirely. Note that a clamped goal still reports
`SUCCEEDED`, which is exactly why the loud layer is still wanted.

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
| Firmware clamp | **done** | `firmware/esp32/include/joint_limits.hpp`. Limits are COMPILED IN, not sent over the wire - a limit the host can set is a limit the host can get wrong. Tested natively by `test_firmware_logic.cpp`, and a test asserts the duplicated values have not drifted from the profile. |
| `RobotClient` validation | **done** | Rejects out-of-limit goals before sending. |
| `ros2_control` limits | **done** | `enforce_command_limits: true` on controller_manager. `ResourceManager` clamps every command below the hardware seam, so it holds for mock, Gazebo and the ESP32 alike. Asserted by `test_ros_integration.py`. |
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

**Partly implemented.** The firmware has a latching stop:

- `kFlagEmergencyStop` cuts torque immediately.
- It **stays latched** when the stop command stops arriving. An e-stop
  that releases because the link dropped is not an e-stop.
- Only an explicit `kFlagClearFault`, not also asserting stop, releases
  it.

All four behaviours are asserted in `test_firmware_logic.cpp`.

**A hardware cut is still missing, and it is the one that matters.** The
above depends on the ESP32 running correct software. A real e-stop is a
switch in the servo power line that works when the firmware has hung, and
no amount of testing substitutes for it.

Until that exists: **keep the power switch within reach before you power
on.**

## What a lost heartbeat does, and why

250 ms of silence from the host (12 missed frames at 50 Hz) trips the
firmware's timeout. It then **holds position** rather than releasing.

Releasing would drop the arm under gravity. On a desk that means the
gripper swinging into whatever is in front of it, so holding is the safer
failure — and the operator still has the power switch, which is the
layer that actually stops things.

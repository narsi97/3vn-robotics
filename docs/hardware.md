# Hardware

> No physical arm exists yet. Every value in `config/*.yaml` is
> `provenance: estimated`. This document describes the intent and the
> seam; Phase 5 makes it real.

BOM and safety: [bom-hardware.md](bom-hardware.md).

## The shape of it

4 DOF plus a gripper:

| Joint | Axis | Range | Servo | Effort limit |
|---|---|---|---|---|
| `shoulder_pan` | Z | −90°…+90° | MG996R | 0.922 N·m |
| `shoulder_lift` | Y | −15°…+120° | MG996R | 0.922 N·m |
| `elbow` | Y | −120°…+10° | MG996R | 0.922 N·m |
| `wrist` | Y | −90°…+90° | SG90 | 0.177 N·m |
| gripper | prismatic | 0…18 mm | SG90 | 10 N |

Effort and velocity limits are **derived from the servo datasheets** at
4.8 V (the BOM specifies a 5 V supply), recorded in the `servos:` block of
each profile. `test_servo_limits.py` fails if any joint is configured
beyond what its servo can deliver — an effort limit above stall torque
lets Gazebo validate motions that stall on the bench, which is the
sim-to-real mismatch this platform exists to catch.

Static check: holding the arm horizontal needs **0.282 N·m** at the
shoulder against a 0.922 N·m limit — **3.3× headroom** unloaded, ~1.7×
with a 100 g payload.

Asymmetric ranges are deliberate — the arm cannot fold back through its
own base.

The right gripper finger follows the left through a URDF `<mimic>` tag,
which is what a single-servo gripper physically does.

## Where the software meets the hardware

One `hardware_interface::SystemInterface` plugin,
`threevn_hardware/Esp32SystemInterface`. The plugin name and the
`target:=esp32` argument are **already fixed in the description**, so
Phase 5 is purely additive: one new package, nothing above it changes.

### The control loop runs on the host

The host runs `ros2_control` at 50 Hz and the ESP32 is a dumb servo
driver behind a small framed binary protocol over USB CDC or TCP.

**micro-ROS is deliberately off the control path.** There are reports of a
~1–2 Hz publish ceiling for micro-ROS on ESP32 — two orders of magnitude
below a usable servo rate. Because the seam is a plugin, the transport is
private to `read()`/`write()` and can change later without touching
anything else. micro-ROS becomes an optional advanced module, not a
dependency.

### The firmware's job, and what it is not

Does: PWM generation, joint limit clamping, heartbeat, e-stop, reporting
firmware version and health.

Does **not**: kinematics, trajectory generation, planning. High-level
intelligence stays in ROS 2, or the "not coupled to the ESP32" property is
lost — and debugging a 4 MB microcontroller is far harder than debugging a
laptop.

### Protocol sketch

Framed binary, versioned, with a CRC. Position command and state frames
at 50 Hz; a heartbeat whose absence triggers hold-position then disable.

The codec will live in its own translation unit so it can be **unit-tested
with no hardware attached** — which is the point of putting the seam here.

## Calibration debt

Servo horns mount at arbitrary angles, so a joint's mechanical zero will
not be its commanded zero. That offset belongs in the profile YAML
alongside every other physical number, and every mass must move from
`provenance: estimated` to `measured` once an arm exists. Tracked in
[roadmap.md](roadmap.md).

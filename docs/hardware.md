# Hardware

> No physical arm exists yet, and every value in `config/*.yaml` is
> still `provenance: estimated`. But the software side of Phase 5 is
> done: the firmware, the protocol and the hardware interface all exist
> and are tested without a device. See [firmware.md](firmware.md).

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

### The firmware's job, and what it is not — now written

`firmware/esp32/`. Does: PWM generation, joint limit clamping, heartbeat,
a latching e-stop, reporting firmware version and health.

Does **not**: kinematics, trajectory generation, planning. High-level
intelligence stays in ROS 2, or the "not coupled to the ESP32" property is
lost — and debugging a 4 MB microcontroller is far harder than debugging a
laptop.

### The protocol — implemented

Framed binary, versioned, CRC-16/CCITT-FALSE, little-endian. Positions
travel as signed micro-units of each joint's SI unit, the same conversion
for every joint, so there is no per-joint scale table to get wrong.

`protocol.hpp` is compiled into **both** the firmware and the hardware
interface, so the two cannot disagree. 20 tests cover it — round-trips,
CRC against the published vector, corrupted payloads and lengths, wrong
versions, frames split across reads, resync after garbage, and 200,000
bytes of noise.

One property worth knowing: a truncated frame costs you the **next** frame
too, because the parser is stranded mid-payload until its CRC fails. That
is inherent — payload bytes can legitimately contain the magic, so
scanning for it inside a payload would resync on data. The loss is
bounded at one frame (20 ms at 50 Hz), and a test asserts it stays
bounded rather than wedging the link.

## Calibration debt

Servo horns mount at arbitrary angles, so a joint's mechanical zero will
not be its commanded zero. That offset belongs in the profile YAML
alongside every other physical number, and every mass must move from
`provenance: estimated` to `measured` once an arm exists. Tracked in
[roadmap.md](roadmap.md).

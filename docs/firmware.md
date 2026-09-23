# ESP32 firmware

```
firmware/esp32/
├── platformio.ini
├── include/joint_limits.hpp    compiled-in limits, the last line of defence
├── include/controller.hpp      heartbeat, e-stop, clamping - no Arduino
└── src/main.cpp                bytes and PWM, nothing else
```

## The split, and why it matters

`controller.hpp` and `joint_limits.hpp` contain **every decision the
firmware makes** — what a servo is told, when to stop trusting the host,
whether an e-stop can be released. They include no Arduino headers, do no
allocation and read no clock: time arrives as a parameter.

That makes them compile on a laptop, which is why
`threevn_hardware/test/test_firmware_logic.cpp` can test 19 behaviours of
this firmware with **no ESP32 attached** — including the ones you would
otherwise verify by flashing and trying it on a powered arm.

`main.cpp` is what remains: read bytes, write PWM. Debugging a 4 MB
microcontroller with no debugger is enormously slower than running a
test, so anything that can be decided off the device is.

## The protocol is not written twice

`platformio.ini` adds `-I../../src/threevn_hardware/include`, so the
firmware and the ROS 2 hardware interface compile **the same**
`protocol.hpp`. Framing, field order and units cannot drift, because
there is only one definition.

A protocol written twice drifts, and the drift shows up as an arm moving
to the wrong place rather than as a compile error.

## Limits are compiled in, on purpose

`joint_limits.hpp` duplicates the joint limits from
`config/threevn_arm_v1.yaml`. That duplication **is** the safety
property: a limit the host can set is a limit the host can get wrong, and
this is the only layer that still works when the host has crashed.

The cost of duplication is drift, so
`FirmwareLimits.MatchTheRobotProfileTheyDuplicate` parses the YAML and
asserts the two agree. Change one without the other and the build fails.

Three layers now clamp, and each covers a different failure:

| Layer | Survives | Added |
|---|---|---|
| `RobotClient` validation | rejects loudly, names the offending joint | Phase 3 |
| `ros2_control` `enforce_command_limits` | any caller, including one bypassing the client | Phase 4 |
| **firmware** | **the host crashing** | Phase 5 |

## Behaviour worth knowing before you power it

- **Nothing moves at boot.** The servos are not attached until the host
  sends an enabled command. A device that energises at boot moves before
  anything has told it where to go.
- **A lost heartbeat holds position, it does not release.** 250 ms of
  silence (12 missed frames at 50 Hz) trips the timeout. Cutting torque
  would drop the arm under gravity — on a desk that means the gripper
  swinging into whatever is in front of it.
- **The e-stop latches.** It is not cleared by the stop command ceasing,
  only by an explicit `kFlagClearFault` that is not also asserting stop.
  An e-stop that releases when the link drops is not an e-stop.
- **Clamping is reported.** `kStatusLimitClamped` tells the host the
  firmware overrode it. Silent disagreement between commanded and actual
  is what makes a robot baffling to debug.

## A real limitation: there is no position feedback

Hobby servos do not report where they are. The device reports its
**target** as `measured`, which means a stalled or stripped servo looks
perfectly healthy from the host.

This is why the host differentiates position for velocity rather than
trusting a number the device invented, and it is the single strongest
argument for feedback servos when the budget allows — see
[bom-hardware.md](bom-hardware.md).

## Building and flashing

```bash
cd firmware/esp32
pio run                 # build
pio run -t upload       # flash
```

PlatformIO is **not** in the dev container: it pulls a large toolchain
that nothing else needs, and flashing requires the USB device anyway.
Install it on the host (`pipx install platformio`).

## Serial ports: two of them, deliberately

`Serial` carries **binary frames only**. Opening a terminal on it shows
garbage, and — more importantly — a stray `print` there corrupts the
protocol.

Diagnostics go to `Serial1` on GPIO17 at 115200. Wire a USB-serial
adapter to read them.

## The macOS USB problem

**Docker Desktop on macOS cannot forward USB devices to a container.**
There is no `--device` equivalent for a USB serial adapter; the Linux VM
does not see the host's `/dev/cu.usbserial-*`.

Three options, in order of preference:

1. **Run the ROS stack on a Linux host or SBC** — a Raspberry Pi next to
   the arm is the intended production shape anyway, and `--device` works
   there. This is what the deployment model assumes.
2. **Bridge over TCP** — run a `socat` TCP↔serial bridge on the Mac and
   point the interface at it. Needs the `tcp` transport, which is
   declared in the config and **not implemented** (`on_init` refuses it
   rather than silently behaving as serial).
3. **A Linux VM with USB passthrough** — UTM or Parallels can forward the
   device.

None of this blocks development: the firmware logic, the protocol and the
hardware interface are all tested without a device. It only affects the
final bring-up on real servos.

## Safety

Read [safety.md](safety.md) before powering anything. The short version:
software limits are not a safety system, the power switch is, and
`make robot` will refuse to start if no device is present rather than
crashing in a way that reads as a bug.

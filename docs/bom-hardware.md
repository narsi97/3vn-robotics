# Bill of materials — hardware

> **No printable parts exist yet.** The repository contains a *kinematic*
> model (URDF primitives with correct lengths, masses and joint axes) but
> no CAD: no STL, no STEP, and `meshes/` is empty. The filament line below
> is therefore a **projected** cost for parts that still have to be
> designed. See [roadmap.md](roadmap.md).

**Two figures, stated separately and on purpose.** Quoting "$30 robot arm"
while excluding the controller, the power supply and the wiring would be a
misrepresentation. The arm can be ~$30. The *system* cannot.

| | Cost |
|---|---|
| **The moving arm** — servos, printed parts, fasteners | **~$30** |
| **Complete working system** — plus controller, driver, PSU | **~$45–55** |

Prices are typical retail at time of writing and vary by region and
supplier. Buying servos in multi-packs moves the arm figure toward $20.

---

## The arm (~$30)

| Item | Qty | Unit | Total | Note |
|---|---|---|---|---|
| MG996R metal-gear servo | 3 | $3–6 | $9–18 | base, shoulder, elbow |
| SG90 / MG90S micro servo | 1 | $2–3 | $2–3 | wrist or gripper |
| PLA filament | ~250 g | $20/kg | ~$5 | **projected** - parts not yet designed |
| M3 hardware, bearings, zip ties | — | — | ~$5 | |
| | | | **~$21–31** | |

## Making it work (+$15–25)

These are **not optional**. An arm with no controller and no power supply
does not move.

| Item | Qty | Unit | Note |
|---|---|---|---|
| ESP32 DevKitC | 1 | ~$6.40 | the controller |
| PCA9685 16-ch servo driver | 1 | ~$7 | see below |
| 5 V 5 A power supply | 1 | ~$10 | see below |
| Dupont wire, barrel jack, capacitor | — | ~$3 | |

**On the power supply.** Do not power servos from the ESP32's USB rail.
Four MG996R servos can draw several amps under stall, which will brown out
the ESP32 and cause resets that look like firmware bugs. A separate 5 V
supply with a common ground is the minimum safe arrangement.

**On the PCA9685.** Strictly optional — an ESP32 has enough PWM channels
for four servos. It is recommended anyway because it isolates servo power
from logic power, gives consistent 12-bit PWM timing independent of what
the ESP32's CPU is doing, and leaves room for the mobile base in Phase 10.

## Optional

| Item | Cost | Phase |
|---|---|---|
| USB camera (any UVC) | $8–15 | 12 — perception |
| Raspberry Pi 4/5 or mini PC | $60–120 | on-robot host, if untethering |
| 4× DC gear motors + driver + chassis | $25–40 | 10 — mobile base |
| IMU (MPU6050 / BNO055) | $3–12 | 10 — odometry |
| LiDAR (RPLIDAR A1) | $80–100 | 11 — navigation |

## What you also need, and probably have

A 3D printer (or a print service — ~$15–25 for the set), a soldering iron,
a multimeter, and a USB-C cable.

---

## Software: $0

Every component is free and open source, and all of it is permissively
licensed — which is what makes the container redistributable.

ROS 2 Jazzy, Gazebo Harmonic, `ros2_control`, RViz 2, Docker, Ubuntu,
Python, GCC. Full list with licences: [`../THIRD_PARTY.md`](../THIRD_PARTY.md).

---

## Safety

**This is an educational hobby robot, not industrial equipment.**

Software limits are not a safety system. `ros2_control` enforces joint
limits, and the firmware will enforce them again in Phase 5, but a servo
with a bug, a brownout, or a mechanical failure does not consult either.

- Keep fingers clear of the gripper and the elbow while powered.
- Power the arm from a bench supply with current limiting if you have one.
- Cut power at the supply, not in software, when something goes wrong.
- Do not leave it powered and unattended.
- MG996R servos have enough torque to hurt, and metal gears do not slip.

Every physical value in the config files is currently `provenance:
estimated`. Re-measure after fabrication — see
[`roadmap.md`](roadmap.md).

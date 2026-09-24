# Third-party components

Everything this project depends on, what it is used for, and whether its
licence permits the use we make of it.

This file exists because 3VN Robotics is intended to become a **paid
course**. That makes licence compatibility a product question, not a
formality: a non-commercial licence anywhere in the dependency graph is a
blocker, not a footnote.

*Nothing here is legal advice.*

---

## Rejected: EEZYbotARM (and EEZYbotARM MK2)

| | |
|---|---|
| Author | daGHIZmo |
| Source | [thing:1015238](https://www.thingiverse.com/thing:1015238), [thing:1454048](https://www.thingiverse.com/thing:1454048) |
| Licence | **CC BY-NC 4.0 — Attribution, NonCommercial** |
| Status | **Not used. No files from it are present in this repository.** |

The EEZYbotARM family was the obvious starting point: a well-known,
genuinely open, 3D-printable 4-axis arm with existing control software,
and the reference design named in this project's original specification.

It was rejected after checking the licence. Both the original and the MK2
carry **CC BY-NC 4.0**, verified by reading the licence field on the live
Thingiverse pages. The NonCommercial term is incompatible with selling a
course built around the design.

Two points that are easy to get wrong, and why the rejection is broader
than "don't ship the STLs":

1. **A derived mesh is still a derivative work.** Re-modelling the arm
   from their CAD, or exporting a simplified collision mesh from it, does
   not launder the licence.
2. **A URDF that reproduces the design's measurements is arguably
   derivative too.** Not obviously so — measurements of a physical object
   are closer to fact than to expression — but "arguably" is the wrong
   amount of certainty to build a commercial product on.

**What we did instead:** 3VN Arm v1 uses our own geometry — primitive
boxes and cylinders — with every dimension in
`threevn_robot_description/config/*.yaml`. See
[ADR-0004](docs/decisions/0004-own-geometry.md).

**If you personally want to build an EEZYbotARM**, you can: printing one
for your own learning is non-commercial use and entirely within the
licence. Download it from Thingiverse under its own terms, measure your
printed arm, and write those numbers into a profile YAML. We link to it;
we do not redistribute it, and we ship no profile derived from its
published dimensions.

---

## Rejected: Annin Robotics AR4 (MK2 / MK3 / MK5)

| | |
|---|---|
| Author | Chris Annin, Annin Robotics |
| Source | [anninrobotics.com](https://anninrobotics.com), [github.com/Annin-Robotics](https://github.com/Annin-Robotics/ar4-hmi) |
| Licence | **Annin Robotics Open Source Non-Commercial License v1.1 (2025)** |
| Status | **Not used. No files from it are present in this repository.** |

Checked because it is frequently recommended as *the* solid open-source
arm, and it is a genuinely impressive piece of engineering: six axes,
machined aluminium, steppers with proper drivers, a real build manual,
and control software with a ROS interface.

It is rejected for this project on two independent grounds, either of
which alone would be sufficient.

### 1. The licence is more restrictive than EEZYbotARM's, not less

Read from the licence file in their own repository. Software (Section 1):

> Permission is hereby granted [...] to use, copy, and redistribute it
> [...] for **non-commercial purposes only**

> **Prohibited Use:** Commercial sale, licensing, or monetization of the
> Software or derivative works thereof is strictly prohibited without
> prior written consent.

Mechanical designs, CAD and print files (Section 2) are tighter still:

> These materials are provided strictly for educational, research, and
> personal non-commercial use.

> **No Redistribution:** Redistribution, republishing, or hosting of
> design files or physical components in any form is strictly
> prohibited.

> **No Commercial Use:** Selling robots, robot parts, kits, derivatives,
> or modified versions of these designs is strictly prohibited without
> explicit written permission.

EEZYbotARM's CC BY-NC 4.0 at least permits redistribution with
attribution for non-commercial purposes. This prohibits redistribution
outright, and names derivatives and modified versions explicitly. A
course sold for money that is built around it is exactly what Section 2
forbids.

**There is a stated commercial path, and it is worth knowing about.**
The licence invites it directly:

> If you wish to incorporate Annin Robotics designs or software into a
> commercial product or service [...] please contact
> info@anninrobotics.com to discuss a separate commercial license.

That is a real option this project does not have with an anonymous
Thingiverse upload. It has not been pursued, and pursuing it would be a
business decision rather than a technical one. (Not legal advice.)

### 2. It is a different machine by a factor of forty

| | |
|---|---|
| AR4 MK3 combo kit | ~$1,189 |
| stepper motors, drivers, PSU | ~$750 |
| gripper | $75–112 |
| **AR4 total** | **~$2,000** |
| **this project's whole robot** | **~$45–55** |

The AR4 is an industrial-class arm that happens to be buildable at home.
This project's arm is four hobby servos and printed plastic. Even with a
commercial licence, adopting it would replace the thing the course is
about — carrying software practice into *affordable* hardware — with a
$2,000 prerequisite.

### What it is still good for

Nothing here stops anyone **building one personally**: that is
explicitly permitted, and it is a better robot than the one in this
repository. It is also worth reading as a reference for how a serious
open arm is documented — its build manual and BOM structure set a bar
this project's own hardware docs should aim at.

What cannot happen is any of its files, meshes, or derived geometry
entering this repository or a paid course built on it.

## Considered: SO-ARM100 / SO-101

| | |
|---|---|
| Author | The Robot Studio / Hugging Face (LeRobot) |
| Source | https://github.com/TheRobotStudio/SO-ARM100 |
| Licence | Apache-2.0 — commercial use permitted |
| Status | Not used for v1. Viable future hardware profile. |

Licence-clean and technically better: bus servos with real position
feedback via magnetic encoders, rather than open-loop hobby servos. But
six Feetech STS3215 servos put the bill of materials around $120–150
against this project's ~$30 target, and bus servos change the ESP32
firmware story substantially.

Revisit if the budget constraint is ever relaxed. Nothing in the software
architecture prevents it — a new profile YAML and a new
`hardware_interface` plugin is the whole change.

---

## Referenced, not depended on

| Project | Licence | Note |
|---|---|---|
| `ghcr.io/marinholab/gazebo:jazzy` | — | A community ROS 2 Jazzy + Gazebo Harmonic arm64 image, useful as a smoke check that headless Gazebo works on a given machine. **Not a dependency:** we build our own image so the version matrix is auditable and pinned. See [ADR-0002](docs/decisions/0002-dev-environment.md). |

---

## Runtime and build dependencies

All are permissively licensed (Apache-2.0 / BSD-3-Clause / MIT) and
redistributable, which is what makes the Docker image shippable.

| Component | Licence | Used for |
|---|---|---|
| ROS 2 Jazzy Jalisco | Apache-2.0 | Middleware, build system, launch |
| Gazebo Harmonic | Apache-2.0 | Physics simulation |
| `ros_gz` | Apache-2.0 | ROS 2 ↔ Gazebo bridge |
| `ros2_control`, `ros2_controllers` | Apache-2.0 | Hardware abstraction, controllers |
| `gz_ros2_control` | Apache-2.0 | `SystemInterface` for Gazebo |
| `xacro` | BSD-3-Clause | URDF macro expansion, `load_yaml` |
| `urdfdom` / `urdf_parser_py` | BSD-3-Clause | URDF parsing in tests |
| RViz 2 | BSD-3-Clause | Visualisation |
| Ubuntu 24.04 (Noble) base image | various, all redistributable | Container base |
| noVNC | MPL-2.0 | Browser access to the virtual display |
| websockify | LGPL-3.0 | WebSocket ↔ VNC bridge |
| x11vnc | GPL-2.0 | VNC server for the virtual display |
| Xvfb, Mesa (llvmpipe) | MIT / MIT | Virtual display, software OpenGL |
| supervisor | BSD-derived | Process supervision in the dev container |
| fluxbox | MIT | Window manager for the virtual display |

**On the GPL/LGPL entries:** `x11vnc` (GPL-2.0) and `websockify`
(LGPL-3.0) appear only in the **`dev`** image stage, as unmodified
distro packages invoked as separate processes. They are not linked into
anything we distribute, and the `ci` and future runtime stages do not
contain them. Using them this way does not impose GPL terms on this
project's own code. If a binary robot image is ever distributed, it
should be built from the `ci` stage or lower, which excludes them
entirely.

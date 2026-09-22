# ADR-0002: Dev environment — one arm64 container, GUI over noVNC

**Status:** accepted · **Date:** 2026-09-22

## Context

Development happens on an Apple M1. ROS 2 Jazzy and Gazebo Harmonic are
Linux software. macOS has no X11 by default, and Docker Desktop provides
no GPU passthrough, so anything using OpenGL must render in software.

## Decision

### Build our own image from `ros:jazzy-ros-base`

Official `ros:jazzy*` images publish native `linux/arm64` manifests, so
the M1 runs them with no QEMU emulation.

Gazebo Harmonic is installed through the **ROS vendor packages** on
`packages.ros.org` (`ros-jazzy-ros-gz` pulls `gz_sim_vendor`,
`gz_tools_vendor`). Jazzy↔Harmonic is the officially paired combination,
so one apt source is sufficient.

**We deliberately do not add `packages.osrfoundation.org`.** Mixing the
two repositories is the standard route to two incompatible sets of
`libgz-*.so`: apt resolves it without complaint and the failure appears
much later as a `gz sim` segfault or a `gz_ros2_control` missing symbol.
CI asserts that no `gz-*` package resolves from osrfoundation.

The community image `ghcr.io/marinholab/gazebo:jazzy` is a fine smoke
check but rejected as a dependency: no stability contract behind a paid
course, unauditable layers for the licence audit, no room for the Phase 5
ESP32 toolchain layer, and a course teaching "you own your environment"
should not hand students a black box.

### GUI over noVNC, not XQuartz

| Option | Verdict |
|---|---|
| **XQuartz + X11 forwarding** | **Rejected.** RViz2's OGRE needs OpenGL 3.3+; XQuartz's indirect GLX caps near 1.4, producing exactly [ros2/rviz#929](https://github.com/ros2/rviz/issues/929) — `OpenGL 1.5 is not supported`. Also needs `xhost +`, which opens the X server to the network. This would be the single largest support burden for course students. |
| **Headless only** | **Insufficient alone.** `--headless-rendering` (OGRE2 + EGL over llvmpipe) is essential for CI and is in the design — but produces no GUI, and a course whose first lesson is "look at your robot" needs pixels. |
| **noVNC in-container** | **Chosen.** Xvfb is a real X server where Mesa llvmpipe exposes **OpenGL 4.5**, so RViz initialises cleanly — precisely what XQuartz cannot do. Reached at `http://localhost:8106/vnc.html`. No host X server, no `xhost +`, identical on M1/Intel/Linux/Windows, and it matches the existing 3VN "browse to a port" model. |

Both live in one image: `ci` stops before the GUI packages and runs
headless; `dev` adds RViz, Xvfb, x11vnc and noVNC.

### One container

Splitting ROS nodes across containers means fighting DDS multicast
discovery, which is not the lesson this project exists to teach. One
container makes `make sim` a single command with zero discovery
configuration. Split in Phase 6, when there is a genuine network boundary
to model.

### Named volumes for colcon output

`build/`, `install/` and `log/` are named volumes, not bind mounts.
colcon writes tens of thousands of small files; through VirtioFS on macOS
that cost dominates every build. It also keeps generated artifacts out of
the host repository.

## Consequences

- Gazebo's GUI runs at roughly 5–15 FPS on llvmpipe. That is a property
  of CPU rendering, not a fault; `make sim` is headless by default and
  `GUI=1` warns.
- Docker Desktop must be given **≥ 6 GB RAM**, or Gazebo is OOM-killed in
  a way that looks like a Gazebo bug. `make doctor` checks this and prints
  the exact Settings path.
- `--symlink-install` means editing a YAML, xacro or launch file takes
  effect with no rebuild. **Adding** a file still needs `make build`.

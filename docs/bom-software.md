# Bill of materials — software

**Total cost: $0.** Every component is free and open source.

More usefully: every component is **permissively licensed**
(Apache-2.0 / BSD / MIT), which is what makes the container
redistributable and the course sellable. Full audit with per-component
licences and the GPL/LGPL analysis: [`../THIRD_PARTY.md`](../THIRD_PARTY.md).

## Version matrix

| Component | Version | Source |
|---|---|---|
| Ubuntu | 24.04 Noble | base image |
| ROS 2 | Jazzy Jalisco | `ros:jazzy-ros-base` |
| Gazebo | Harmonic (`gz sim` 8.15.0) | ROS vendor packages, `packages.ros.org` |
| `ros_gz` | Jazzy release | apt |
| `ros2_control` / `ros2_controllers` | Jazzy release | apt |
| `gz_ros2_control` | Jazzy release | apt |
| Python | 3.12 | Noble |
| Mesa (llvmpipe) | 25.2.x — reports **OpenGL 4.5** | Noble |
| Docker | 29.x | host |

ROS 2 Jazzy pairs with Gazebo Harmonic **by default**, which is why both
come from one apt source. See [docker.md](docker.md) for why adding a
second source breaks things.

## Why these versions

**Jazzy** is the current LTS, supported to 2029, and the last release
where Gazebo arrives as ROS vendor packages rather than a separate repo —
which removes an entire class of ABI mismatch.

**Harmonic** is the paired Gazebo. It supports `--headless-rendering`
(OGRE2 + EGL), which renders camera sensors with no GPU and no X server —
the single capability that makes CI and an M-series Mac both viable hosts.

**arm64 throughout.** Every image here has a native `linux/arm64`
manifest, so development on Apple Silicon needs no emulation.

## Tooling

| Tool | Purpose |
|---|---|
| colcon | build |
| pytest + `ament_cmake_pytest` | tests |
| `ament_flake8`, `ament_pep257`, `ament_copyright`, `ament_xmllint` | lint |
| `urdf_parser_py` | URDF parsing in tests (Python) |
| `check_urdf` (`liburdfdom-tools`) | independent C++ parser cross-check |
| numpy | inertia tensor algebra |
| Xvfb + x11vnc + noVNC + fluxbox | GUI without a host X server |
| supervisor | process supervision in the dev container |

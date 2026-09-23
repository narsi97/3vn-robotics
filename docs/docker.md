# Docker

One Dockerfile, six stages, two shipping images.

```
base      ROS 2 Jazzy + ros2_control. No simulator, no GUI, no build tools.
  ├── simdeps   + Gazebo Harmonic
  │     ├── ci    + pytest, linters, Mesa       what CI runs
  │     │     └── dev   + RViz, Xvfb, noVNC     what you run       4.96 GB
  │     └── builder + a built workspace, robot packages only
  └── runtime   base + that build               what ships         1.67 GB
```

## The robot image carries no simulator

`runtime` inherits from `base`, which never installs Gazebo. That is only
possible because `threevn_bringup` and `threevn_hardware` genuinely do
not depend on it — the architectural rule
`test_package_dependencies.py` has enforced since Phase 1.

The `builder` stage makes it concrete:

```
colcon build --merge-install \
  --packages-up-to threevn_bringup threevn_hardware threevn_dashboard
```

`threevn_sim` is simply not built. **4.96 GB → 1.67 GB**, a third of the
size, and `make runtime-verify` asserts the simulator is absent rather
than trusting it.

It also settles a licence question. `x11vnc` (GPL-2.0) and `websockify`
(LGPL-3.0) exist **only** in `dev`. Nothing distributed to a robot
contains them — asserted in the same check. See
[`../THIRD_PARTY.md`](../THIRD_PARTY.md).

### One honest caveat

`hardware_interface` depends on `sdformat_urdf`, which pulls about 8 MB
of Gazebo *vendor libraries* and the generic `gz` CLI. So `gz` exists in
the runtime image — but `gz sim` does not, and neither does
`gz_ros2_control` or any bridge. "No simulator" is accurate; "no Gazebo
code at all" would not be.

## The runtime user needs a writable home

Unlike the other 3VN app images, this one creates one. `ros2 launch`
builds a log directory under `$HOME/.ros` before it will start; with
`--no-create-home` it dies inside `launch.logging.get_logger()` with a
traceback that never mentions permissions. `ROS_LOG_DIR` is set
explicitly so the location is predictable.

## Where Gazebo comes from, and the trap

Gazebo Harmonic is installed via the **ROS vendor packages** on
`packages.ros.org` (`ros-jazzy-ros-gz` pulls `gz_sim_vendor`,
`gz_tools_vendor`). Jazzy↔Harmonic is the officially paired combination,
so one apt source suffices.

**Never add `packages.osrfoundation.org`.** Mixing the two repositories
gives you two incompatible sets of `libgz-*.so`. apt resolves it without
complaint, and the failure appears much later as a `gz sim` segfault or a
`gz_ros2_control` missing symbol — hours to diagnose, trivial to avoid.

Both `make doctor`'s sibling checks and a dedicated CI step assert this:

```bash
apt-cache policy ros-jazzy-gz-sim-vendor | grep -c osrfoundation   # must be 0
```

## Architecture

No `--platform` is pinned. `ros:jazzy-ros-base` publishes a native
`linux/arm64` manifest, so the image builds natively on Apple Silicon and
natively on x86 CI. Pinning one would force QEMU emulation on the other
and turn a 5-second colcon build into minutes.

```bash
docker image inspect threevn-robotics-dev:local --format '{{.Architecture}}'
# arm64 on an M-series Mac
```

## Why named volumes for colcon output

`build/`, `install/` and `log/` are named volumes, not bind mounts.
colcon writes tens of thousands of small files, and through VirtioFS on
macOS that I/O dominates every build. It also keeps generated artifacts
out of the host repository.

`src/` *is* bind-mounted, because that is what you edit.

## Two ways ROS gets sourced

Both are needed; neither is redundant:

- `docker/entrypoint.sh` — covers `docker compose run` / `up`.
- `/etc/profile.d/10-ros.sh` — covers `docker compose exec`, which
  bypasses ENTRYPOINT entirely.

Without the second, `make build` works and `make shell` mysteriously does
not.

## Resources

Docker Desktop must have **≥ 6 GB RAM** and ideally 4+ CPUs. Below that
Gazebo is OOM-killed in a way that looks like a Gazebo bug. `make doctor`
checks and prints the exact Settings path.

Budget roughly **10 GB** of disk: ~5–6 GB image plus build volumes and
cache.

## Healthcheck

```yaml
test: ["CMD", "curl", "-fsS", "http://127.0.0.1:8080/vnc.html"]
```

`127.0.0.1`, never `localhost` — house rule across all 3VN products.
websockify binds IPv4 only, and a container's `/etc/hosts` resolves
`localhost` to `::1` first, so a `localhost` healthcheck fails against a
perfectly healthy service.

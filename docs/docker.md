# Docker

One image, three stages.

```
base   ROS 2 Jazzy + Gazebo Harmonic. No GUI. Nothing optional.
  ↓
ci     + pytest, ament linters, Mesa software GL. What CI runs.
  ↓
dev    + RViz, Xvfb, x11vnc, noVNC, supervisor. What you run.
```

Splitting this way means CI tests the same base layers you develop on,
without paying for a GUI stack it never uses. It also keeps the future
robot runtime image clean: build from `ci` or lower and it contains no
`x11vnc` (GPL-2.0) or `websockify` (LGPL-3.0) — see
[`../THIRD_PARTY.md`](../THIRD_PARTY.md).

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

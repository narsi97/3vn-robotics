# Troubleshooting

Symptoms that mislead, and what they actually mean.

## Gazebo dies, or "the simulator just stops"

**Almost always the Docker VM ran out of memory.** The kernel OOM-kills
the process and Gazebo leaves no useful message, so it reads as a crash.

```bash
make doctor                      # reports VM memory
docker stats --no-stream         # what is consuming it
```

Fix: Docker Desktop → Settings → Resources → Memory ≥ 6 GB.

Also check for unrelated containers sharing the VM — a stray database or a
Kubernetes-in-Docker node can take a meaningful slice of a small VM.

## RViz: "OpenGL 1.5 is not supported"

You are rendering through XQuartz or a forwarded X display, not the
container's own virtual display. XQuartz's indirect GLX caps near OpenGL
1.4; RViz's OGRE needs 3.3+.

Fix: do not set `DISPLAY` to your host. Use `make view` and the browser at
`http://localhost:8106/vnc.html`. Inside the container `DISPLAY` must be
`:1`.

## The Gazebo GUI is very slow

Expected. Rendering is on llvmpipe — a CPU software rasteriser — because
Docker Desktop gives containers no GPU. Five to fifteen FPS is normal.

`make sim` is headless by default for this reason. `GUI=1` is opt-in.
Physics is unaffected; only rendering is slow.

## `make urdf` fails with a `NameError`

Almost certainly the xacro conditional quoting. This is wrong:

```xml
<xacro:if value="${target == 'mock'}">
```

It substitutes a bare token, which Python then tries to resolve as a
variable. The correct form quotes the substitution:

```xml
<xacro:if value="${'$(arg target)' == 'mock'}">
```

`test_ros2_control_targets.py` exercises all three targets, so this fails
loudly in CI rather than subtly at runtime.

## A YAML change had no effect

Two possibilities.

1. **You added a file rather than editing one.** `--symlink-install`
   symlinks files that existed at build time. Run `make build`.
2. **You edited the installed copy.** `$(find ...)` resolves to
   `install/share`, not `src`. With `--symlink-install` that is a symlink
   back into `src`, so editing either works — but a full `make clean`
   followed by a build from a stale source tree will not.

## `gz sim` segfaults, or a `gz_ros2_control` symbol is missing

The classic signature of **two incompatible sets of `libgz-*.so`** —
almost always because `packages.osrfoundation.org` was added alongside the
ROS vendor packages.

```bash
make shell
apt-cache policy ros-jazzy-gz-sim-vendor | grep -c osrfoundation   # must be 0
```

Gazebo must come from `packages.ros.org` only. See
[ADR-0002](decisions/0002-dev-environment.md).

## Port 8106 already in use

```bash
lsof -nP -iTCP:8106 -sTCP:LISTEN
```

The other 3VN products use 8083/8085/8087/8100/8102/8104. If something
else has taken 8106, change the host side of the port mapping in
`docker/compose.yml`.

## Out of disk

The image is 5–6 GB and build cache grows quickly.

```bash
docker system df            # where it went
make nuke                   # this project's image, volumes and cache
docker builder prune -a     # all build cache, every project
```

## Controllers do not start

Check ordering — the broadcaster must be active before the trajectory
controller.

```bash
make shell
ros2 control list_hardware_components   # expect 1, state: active
ros2 control list_controllers           # expect 3, all active
```

In Gazebo, `controller_manager` lives inside the simulator process, so it
does not exist until the model has spawned. The launch file chains
spawners off the spawn event for exactly this reason.

# Getting started

You need **Docker and nothing else**. No ROS, no colcon, no Gazebo, no
Python on your machine.

## 1. Get the code

```bash
git clone https://github.com/narsi97/3vn-robotics.git
cd 3vn-robotics
```

Every command below runs from this folder. The Makefile at its top is
the interface to everything in the course.

## 2. Check the machine

```bash
make doctor
```

Checks the daemon, architecture (native vs emulated), Docker VM memory and
CPUs, free disk, port 8106, and `git`/`gh`. Every failure prints the exact
fix rather than a diagnosis.

**The one that catches most people:** Docker Desktop defaults to less
memory than Gazebo needs. Set **Settings → Resources → Memory to 6 GB or
more**. Below that, Gazebo is OOM-killed by the VM and the symptom looks
like a Gazebo bug rather than a resource limit.

## 3. Build

```bash
make setup
```

Builds the image (ROS 2 Jazzy + Gazebo Harmonic, several GB — this takes a
while the first time) and then the ROS workspace. Subsequent runs are
cached.

## 4. Prove it works

```bash
make test
```

The description test suite. No simulator, no hardware, budgeted to stay
under 60 seconds — because a slow suite is a suite nobody runs before
committing.

## 5. Look at the robot

```bash
make view
```

Then open **http://localhost:8106/vnc.html**. RViz appears with the arm
and a joint-slider window. Drag a slider; the arm moves within its
configured limits.

There is no X server on your Mac and no GPU in the container — the display
is virtual, rendered in software, and served to your browser. See
[ADR-0002](decisions/0002-dev-environment.md) for why this rather than
XQuartz.

## 6. Drive the whole robot

`make view` shows the arm alone. For the robot this project actually
builds towards -- arm on a mobile base, with a camera:

```bash
make play          # headless Gazebo + RViz  -> http://localhost:8106/vnc.html
make drive         # in a SECOND terminal: keyboard teleop
```

`w a s d` drives, `1..4` picks an arm joint, `,` and `.` jog it, `[` and
`]` work the gripper. Let go and the robot stops -- because the
controller brakes on a stale command, not because teleop told it to.

It runs at roughly **a third of real time** on an M1 with no GPU. That is
CPU physics, not a fault. If the view itself is laggy that is a separate
problem with a better fix -- see [viewing.md](viewing.md).

## 7. See the seam

The point of the whole architecture:

```bash
make urdf TARGET=gz    | grep -c GazeboSimSystem   # 1
make urdf TARGET=mock  | grep -c gz_ros2_control   # 0
```

The same description, the same code, a different execution target — and a
build destined for real hardware contains no trace of the simulator.

## Everyday commands

```bash
make              # full command list
make shell        # a shell inside the container, ROS already sourced
make build        # rebuild the workspace after adding a file
make lint         # linters + check_urdf across every profile
make urdf         # print the expanded URDF
make clean        # drop build/install/log
make nuke         # clean + remove the image and volumes
```

`--symlink-install` means editing a YAML, xacro or launch file takes
effect immediately. **Adding** a file still needs `make build`.

## When something breaks

See [troubleshooting.md](troubleshooting.md).

# Stage 1 recording: your environment

The recording for stage 1 of the course. No robot moves in it: the
whole session is one terminal, and the subject is the toolchain.

| | |
|---|---|
| Length | About 21 minutes finished, or six clips of 2½ to 4½ minutes |
| Covers | Stage 1, "Your environment": all six lessons |
| Robot | None runs. The 3VN arm's description is read and checked |
| Needs | Docker, an internet connection, a fresh clone of `main` |
| Recorded in | English. Dubbed and subtitled afterwards; see [terms.yaml](terms.yaml) |

Record the [pilot](pilot-one-command-three-backends.md) first. It is
shorter, and it is the test of the dubbing process. This session uses
the same rules for the picture and the voice, and does not repeat them.

## What the viewer leaves with

One sentence: **the environment is built from one file, and its checks
are trusted because we watched one of them fail.**

## One segment, one lesson

Segments 1 to 6 are the six lessons of the stage, in order. Each one
shows that lesson's own steps and stops, so each can be published alone
on its lesson page. The two `[face]` segments belong to the full-length
version only.

The written lesson still carries what a video cannot: the pitfalls, the
quiz and the exercise. Do not add them to the narration.

Where a segment types something different from the lesson, it says so,
and [the list at the end](#where-the-script-departs-from-the-lessons)
says why.

## Before recording

**The machine**

- Nothing else building or testing on it.
- Online, with Docker able to reach Docker Hub. `make setup` asks
  Docker Hub about the base image on every run, cached or not. Run
  `make setup` to the end once before you record. If it stops at
  `load metadata for docker.io/…` with `DeadlineExceeded`, Docker is not
  getting an answer, and the take cannot be made until it does. That
  happened on this Mac on the day the script was checked.
- `make stop`, so nothing is running in the container.

**The folder**

Record in a fresh clone, so that the first command on screen is real.

Every checkout of this repository on one machine shares a single dev
container and a single set of build volumes. So `make setup` in the new
clone points that container at the new clone. Nothing else on the
machine should be using the container while you record. Afterwards,
`make up` from your usual checkout points it back. (This was not run
here, because other work was using the container. It is how the
container came to follow each checkout so far.)

**The picture**

- 1920×1080, 30 fps. Notifications off.
- One terminal, full width. Font 18 pt or larger, a short prompt.
- An editor for one change in segment 3, at the same font size.
- No browser. Nothing in this stage draws a robot.
- No text on screen in any language but English, and no burned-in
  captions. The reason is in the pilot's notes.

**The voice**

The pilot's rules: short sentences, one idea each, no idioms, name the
thing, do not read commands aloud, two seconds of silence before each
command and three after each result.

## The session

`[face]` is you on camera, or an avatar. `[screen]` is screen capture
with your voice over it. Times are finished length, after cuts.

### 0. The promise `[face]` 0:30

> To follow this course you install one thing: Docker. You do not
> install ROS. You do not install a simulator.
>
> In the next twenty minutes this laptop gets a complete robot
> toolchain. It runs about nine hundred checks on a robot that does not
> exist yet.
>
> Then I break the robot's configuration on purpose. We will see which
> check notices, and what it says.

### 1. Docker, and nothing else `[screen]` 3:30

Lesson: *Docker, and nothing else*.

```bash
git clone https://github.com/narsi97/3vn-robotics.git && cd 3vn-robotics
```

About one second; the repository is 4 MB.

```bash
make
```

Shows the list of targets, from `make doctor` to `make nuke`. Hold it
for three seconds.

> This list is the interface to the whole project. Almost everything
> in the course starts with one of these commands.

```bash
make doctor
```

Shows, in under two seconds:

```
3VN Robotics -- environment check

  ok    docker daemon reachable (Docker version 29.5.3)
  ok    architecture aarch64, native (no emulation)
  ok    docker VM memory 8GB
  ok    docker VM cpus 8
  ok    disk 26GB free
  ok    port 8106 held by our own container
  ok    git 2.50.1
  ok    gh 2.103.3

  Environment looks good. Next: make setup
```

On a machine where the container is not running yet, the port line
reads `port 8106 free`. Both are a pass.

> Before building anything, check the machine.
>
> Memory is on this list for a reason. When Docker has too little
> memory, the simulator is stopped without a message. It looks like a
> fault in the simulator. It is a setting.
>
> When a check here fails, it prints the setting to change.

```bash
make setup
```

**Cut the wait.** With the image already built this is a re-run: Docker
finds every layer cached, and the workspace build ends

```
Summary: 11 packages finished [6.23s]

  Ready. Try:  make test   then   make view   then open http://localhost:8106/vnc.html
```

This is the one step that did not finish when the script was checked;
[the last section](#what-was-checked) says what was run instead.

> This is the long step, and only the first time. It builds one image
> from one file: ROS 2, Gazebo, and every tool the course uses. The
> first build downloads several gigabytes. Later runs reuse them.
>
> The environment is something we build, not something we install by
> hand. You, I and the CI server build it from the same file. So we get
> the same environment.

```bash
docker image inspect threevn-robotics-dev:local --format '{{.Architecture}}'
```

Shows `arm64` on an Apple Silicon Mac, `amd64` on most other machines.

> One more check. The image is built for this machine's own processor.
> Nothing is emulated, so nothing is slow without a reason.

### 2. Living inside the container `[screen]` 3:00

Lesson: *Living inside the container*.

```bash
make shell
```

The prompt changes to `dev@…:/ws$`. Inside it, one line at a time:

```bash
printenv ROS_DISTRO
gz sim --versions
ls -l /ws/install/threevn_robot_description/share/threevn_robot_description/config/
```

Shows `jazzy`, then `8.15.0`, then one line per robot profile, each an
arrow back into the source folder:

```
threevn_arm_v1.yaml -> /ws/src/threevn_robot_description/config/threevn_arm_v1.yaml
```

Hold the listing for five seconds; the arrows are the point.

> This prompt is inside the container. ROS is already loaded.
>
> These are the two versions the whole project is built on: ROS 2
> Jazzy, and Gazebo 8, which is also called Harmonic.
>
> Now look at the arrows. Every installed file is a link back to the
> source folder. So when you edit one of these files, the change is live
> the next time you start the robot. Nothing is rebuilt.
>
> A new file is different. It has no link yet. A new file needs one
> build before anything can see it.

```bash
printenv ROS_AUTOMATIC_DISCOVERY_RANGE
exit
```

Shows `LOCALHOST`.

> ROS finds other robots on the network by itself. On a shared network,
> that means your laptop can join a colleague's robot, and nobody asked.
> This setting keeps discovery inside this one container.

Back on the host:

```bash
docker volume ls | grep -E "_(build|install|log)$"
```

Shows three volumes: `docker_build`, `docker_install`, `docker_log`.

> Your source folder is shared with the container, so your own editor
> works on it.
>
> The build output is not shared. It lives in three volumes that Docker
> manages. A build writes tens of thousands of small files, and on a Mac
> that is much faster inside Docker's own storage.

### 3. A test suite people actually run `[screen]` 4:30

Lesson: *A test suite people actually run*.

```bash
time make test
```

Forty-nine seconds in a new clone. Play the middle at eight times
speed, and slow down for the last three lines:

```
Summary: 11 packages finished [38.3s]
  1 package had stderr output: threevn_control
Summary: 928 tests, 0 errors, 0 failures, 31 skipped
```

The `stderr output` line is there on every run and is not a failure.
In a folder where `make so101-fetch` has been run, eleven more tests
run: `20 skipped`, and 54 to 56 seconds.

> About nine hundred checks, in under a minute. There is no simulator
> running, and there is no robot.
>
> The limit for this suite is sixty seconds. That is a rule, not a
> hope. A suite that takes five minutes is a suite nobody runs before a
> commit.

In the editor, open
`src/threevn_robot_description/config/threevn_arm_v1.yaml`. On line 27
change

```yaml
  model: threevn_arm_v1
```

to

```yaml
  model: 3vn_arm_v1
```

Save. Back in the terminal:

```bash
make test
```

Forty-one seconds, and it ends in red. Speed through the run again and
stop on these lines, which are near the end:

```
FAILED ../../src/threevn_robot_description/test/test_config_schema.py::test_model_name_is_ros_legal[threevn_arm_v1]
```

```
AssertionError: model '3vn_arm_v1' is not a legal ROS identifier -- it must start with a letter (see docs/decisions/0001-package-naming.md)
```

```
Summary: 928 tests, 0 errors, 2 failures, 31 skipped
make: *** [test] Error 1
```

One fault is counted as two failures: the test itself, and the group
of tests it runs in. Do not explain that on camera.

> I changed one name. It now starts with a digit.
>
> The file is still valid YAML. Nothing complains when it is saved.
> The trouble comes later, when ROS builds its own names from this one.
> A name like this can fail deep inside ROS, far away from this file.
>
> The check found it now, and it says what the rule is.
>
> A check that you have never seen fail is a check you trust without
> evidence. Now there is evidence.

```bash
git checkout src/threevn_robot_description/config/threevn_arm_v1.yaml
make test
```

Under a minute again, ending
`Summary: 928 tests, 0 errors, 0 failures, 31 skipped`. Speed through
it, and say nothing until the summary is on screen.

> Back to green.

### 4. The decision everything else rests on `[screen]` 3:00

Lesson: *The decision everything else rests on*.

```bash
make urdf TARGET=gz 2>/dev/null | grep -c GazeboSimSystem
make urdf TARGET=mock 2>/dev/null | grep -c gz_ros2_control
```

Shows `1`, then `0`. Each takes about seven seconds, because the
workspace is checked first. Cut those seconds.

> The software that moves the robot does not know what it is moving. A
> simulator, nothing at all, or real motors. One argument chooses.
>
> Built for the simulator, the robot's description names the
> simulator's plugin. Once.
>
> Built for the mock target, the simulator is not mentioned at all.
> Zero.
>
> The zero is the interesting number. It does not say "we avoid the
> simulator". It says "the simulator cannot be reached from here". A
> real robot should not need a simulator installed.

```bash
make urdf TARGET=esp32 2>/dev/null | grep -c "<gazebo"
make urdf TARGET=esp32 2>/dev/null | grep -c Esp32SystemInterface
```

Shows `0`, then `1`.

> The same holds for real hardware. No simulator, and one plugin for
> the motors.

```bash
grep -n "xacro:if.*target" src/threevn_robot_description/urdf/ros2_control/arm.ros2_control.xacro
```

Shows exactly three lines:

```
78:        <xacro:if value="${'$(arg target)' == 'mock'}">
83:        <xacro:if value="${'$(arg target)' == 'gz'}">
87:        <xacro:if value="${'$(arg target)' == 'esp32'}">
```

> Here is where the choice is made. Nothing at all, the simulator, or
> real motors. Three branches, and no others.
>
> Everything later in this course plugs in at this one place: the
> simulator, the real motors, and a robot that somebody else designed.

### 5. Before the first line, a licence and a name `[screen]` 2:30

Lesson: *Before the first line, a licence and a name*.

```bash
grep "^## " THIRD_PARTY.md
```

Shows, first:

```
## Rejected: EEZYbotARM (and EEZYbotARM MK2)
## Rejected: Annin Robotics AR4 (MK2 / MK3 / MK5)
## Considered: SO-ARM100 / SO-101
```

> The first decision in this project was not about code. It was about
> a licence.
>
> Three arms were checked. Two were rejected. The first one was the arm
> this project was planned around, and its licence does not allow
> commercial use. Checking took minutes, on the first day. That is much
> cheaper than finding out after the work is done.
>
> The third arm, the SO-101, has a licence that allows it. You will
> meet that arm later.

```bash
grep -n -E "model:|display_name:" src/threevn_robot_description/config/threevn_arm_v1.yaml
```

Shows:

```
27:  model: threevn_arm_v1
33:  display_name: "3VN Arm v1"
```

> The second decision was a name. A name in ROS must start with a
> letter. Our name starts with a digit.
>
> So there are two lines. One name is for the machine. One name is for
> people.

```bash
grep -n -A10 "def test_model_name_is_ros_legal" src/threevn_robot_description/test/test_config_schema.py
```

Shows the test, from line 117. Its comment states the rule; the
assertion is one line below where the listing stops.

> And this is the check that failed a few minutes ago. A rule that
> matters gets a test.
>
> Both decisions are written down, with the options that lost and the
> reason each one lost. That part stops the same argument from
> happening twice.

### 6. Symptoms that lie `[screen]` 3:00

Lesson: *Symptoms that lie*.

```bash
grep "^## " docs/troubleshooting.md
```

Shows eleven headings, from `Gazebo dies, or "the simulator just
stops"` to `Controllers do not start`.

> These are failures this project has already paid for. Every one of
> them looked like something else.
>
> The simulator stops without a message: the cause is memory. A command
> lists nothing while the robot is running: the cause is a file left
> behind by a process that was stopped.
>
> The habit is the same every time. The symptom is in one layer, so
> check the layer below it. And confirm with a command, not with a
> theory.

```bash
docker stats --no-stream
docker system df
lsof -nP -iTCP:8106 -sTCP:LISTEN
```

Shows the container's memory against its limit (here `566MiB /
7.75GiB`), Docker's disk use by kind, and one `com.docke…` line
listening on `127.0.0.1:8106`.

> Three commands cover most of that list. Memory. Disk. And the one
> port this project uses.

```bash
make shell
apt-cache policy ros-jazzy-gz-sim-vendor | grep -c osrfoundation
exit
```

Shows `0`, and then, as the shell closes:

```
What's next:
    Debug this Compose error with Gordon → docker ai "help me fix this compose error"
make: *** [shell] Error 1
```

**Keep this in the take.** Nothing failed. `grep -c` found no match,
which is the right answer, and it reports "no match" as exit code 1.
`exit` hands that code to `make`, and `make` prints it as an error.
Docker Desktop then offers to help debug it.

> Zero is the right answer. Gazebo comes from exactly one source. Two
> sources is how the simulator comes to crash.
>
> Now read the last lines. They say "error". Nothing failed. The search
> found no match, which is what we wanted. "No match" leaves as an
> error code, and each tool above it reports an error.
>
> That is a symptom that lies, in the lesson about them.

### 7. What this did not prove `[face]` 0:45

> Nothing moved today. No simulator ran, and no motor. Every check so
> far reads files. That is why nine hundred of them fit in under a
> minute.
>
> It is also their limit. Every file can be correct, and the robot can
> still fall over.
>
> Next is the robot itself: what it is made of, written so that a
> machine can check it.

## After recording

The same four steps as the pilot, with this file's name:
`stage-1-your-environment.en.srt` first, then one reviewed translation
per language, then the dub.

If the segments are also published as six clips, cut the transcript at
the same points and name the parts `stage-1-your-environment.1.en.srt`
to `….6.en.srt`.

## Where the script departs from the lessons

Running every lesson step on `main` found these. The script uses what
works today; the lessons are not changed by this file.

| Lesson | The lesson says | On `main` | This script |
|---|---|---|---|
| 1 | `make` lists targets "from `doctor` and `setup` to `play`, `train` and `site`" | `train` is a target and is not in the list | says "from `make doctor` to `make nuke`" |
| 2 | `docker volume ls \| grep 3vn-robotics` shows three volumes | It prints nothing. The volumes are `docker_build`, `docker_install` and `docker_log`: Compose names them after the `docker/` folder, not the repository | matches the three endings |
| 3 | "Several hundred" assertions, "well under a minute" | 928 tests, about 900 of them run in a new clone, in 41 to 56 seconds against a limit of 60 | says "about nine hundred" and "under a minute" |
| 4 | `make urdf TARGET=gz \| grep -c …` prints `1` | It does, after about 290 lines of build output | adds `2>/dev/null` |
| 4 | `grep -n "xacro:if" …`: "three branches and no others" | It prints fourteen lines: the three targets, four joint-type conditions, and every closing tag | matches the three target lines |
| 6 | `make shell`, then the `grep -c`, expecting `0` | `0`, and then `make: *** [shell] Error 1` when the shell is left | keeps the error and explains it |

One more, read and not run: `make nuke` removes volumes named
`3vn-robotics_build`, `3vn-robotics_install` and `3vn-robotics_log`.
Those are not the names on `main`, so the build volumes would be left
behind. Lesson 1 sends a learner who is out of disk to `make nuke`.

## What was checked

On 2026-11-16, on `main` at `99cb2a4`:

- `git clone`, `make` and `make doctor` were run in a new, empty
  folder.
- Every other command above except `make setup` was run as written, in
  a checkout whose `src/` is `main`'s plus one RViz file from the pilot
  branch. The outputs and timings quoted are from those runs. The
  commands inside `make shell` went to a real interactive shell.
- The fault in segment 3 was planted and reverted for real, twice. With
  the SO-101 description absent, as in a new clone: 0 failures in 49
  seconds, then 2 failures naming the rule in 41. With it fetched: 0
  failures in 56 seconds, 2 failures in 54, and 0 again in 54.

Not checked:

- **`make setup` did not finish.** It was run twice. Both times Docker
  waited two minutes, twice, for Docker Hub to answer about the build
  front end, then gave up on the base image:
  `DeadlineExceeded` after five minutes. The Mac itself reached Docker
  Hub in 0.2 seconds, and so did a container. The cause was not found.
  What was run instead: `make doctor`, which is setup's first step, and
  `make build`, which is its last (7 seconds). The `Ready.` line is
  quoted from the Makefile.
- A first build, on a machine that has never built the image. No time
  for it is given anywhere in this script.
- `make setup` in a second clone, and what it does to the shared
  container.
- The session has not been run end to end in one sitting, and nothing
  has been recorded, transcribed or dubbed.

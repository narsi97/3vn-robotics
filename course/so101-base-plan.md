# The SO-101 base: plan

A working note, not a lesson. It lives in `course/` because the site
builder renders every file in `docs/` and only `*.yaml` from here.

The decision it serves (2026-11-14 and 2026-11-15): the course may run
entirely on an SO-101, the SO-101 follower's servos are ordered and its
parts are printing, and until the arm exists the course is built against
a **virtual SO-101**. The first sessions with the physical arm will be
recorded.

## 1. What exists now

The SO-101 is a profile like any other. Everything that takes `PROFILE`
takes it:

```bash
make so101-fetch                         # once: pinned, checksummed, into vendor/
make sim      PROFILE=so101_follower     # Gazebo (make so101 is the short form)
make mock     PROFILE=so101_follower     # no simulator
make scenario NAME=all                   # the same six scenarios, unchanged
make test-ros PROFILE=so101_follower     # the integration tier against it
make test-sim PROFILE=so101_follower     # the Gazebo tier against it
make urdf     PROFILE=so101_follower     # the description that actually runs

make fake-bus                                         # six simulated servos, on a TCP port
make robot    PROFILE=so101_follower BUS=fake         # the real hardware interface, against them
make test-ros PROFILE=so101_follower TARGET=feetech   # the integration tier, through that interface
```

What was built to get there:

| Piece | Where |
|---|---|
| The profile: an overlay on upstream's URDF (arm joints, gripper travel, named poses, servo data, targets) | `src/threevn_robot_description/config/so101_follower.yaml` |
| `describe()`: the one function every launch file and integration test gets a description from | `src/threevn_robot_description/threevn_robot_description/describe.py` |
| Named poses out of the client and into the profile, carried in the description as `<threevn_semantics>` | `urdf/arm/semantics.xacro`, `robot_client.py` |
| Controllers for the SO-101, on the same names and types as the 3VN arm's | `src/threevn_bringup/config/so101_controllers.yaml` |
| Scenarios that name no joint, pose value or gripper width | `threevn_control/scenarios/arm.py` |
| The upstream description pinned to a commit and checksummed | `scripts/fetch_so101.sh` |
| The servo protocol, checked against packets worked out by hand | `threevn_hardware/feetech_protocol.hpp` |
| `FeetechSystemInterface`: the `feetech` target | `threevn_hardware/src/feetech_system_interface.cpp` |
| A simulated servo bus: six STS3215s, in-process for unit tests and behind TCP for the whole stack | `threevn_hardware/test/support/fake_feetech_bus.hpp`, `make fake-bus` |
| A TCP transport, which is also how a Mac reaches the real bus | `threevn_hardware/src/tcp_transport.cpp` |

Measured in this container on 2026-11-15 (Apple Silicon, CPU physics),
on the final code, by exit code:

| | Result |
|---|---|
| `make test` | 928 tests, 0 failures, 20 skipped (cppcheck, which skips itself), exit 0 |
| `make test-ros PROFILE=so101_follower` | 14 passed, 1 skipped with its reason, exit 0 |
| `make test-ros PROFILE=so101_follower TARGET=feetech` | 14 passed, 1 skipped, exit 0: the same tests, through the real hardware interface and a simulated servo bus |
| `make test-ros` (3VN arm) | 18 passed, exit 0 (15 before this work; 3 scenario tests added) |
| `make test-sim PROFILE=so101_follower` | 12 passed, exit 0 |
| `make test-sim` (3VN arm) | 12 passed, exit 0 |
| `make lint` | 384 tests, 0 failures; 15 of 15 descriptions through `check_urdf`; exit 0 |
| `make runtime-verify` | exit 0 on the image built from this code: no simulator, no simulated hardware, control stack active, `home` passes inside it. Three later rebuilds stopped before building, on Docker failing to pull its own Dockerfile frontend. |
| Real-time factor at idle, upstream collision meshes | 0.11 to 0.13 (three samples) |
| Real-time factor at idle, bounding-box collisions | 0.91 to 1.03 (four samples) |

Those are quiet-machine results, and that qualification is real. With
another session building in a second container (host load average 14 to
31 on 8 cores) the integration tiers failed five times: twice a CLI
helper heard no `/joint_states` inside its timeout, once the same for
`/clock`, and once each the simulator's real-time-factor floor and its
publish-rate floor tripped. Every one passed when re-run alone. The
`/joint_states` helper now asks three times and fails with a message
instead of a `StopIteration`; the timing floors were left as they are,
because a simulator at a quarter of real time is exactly what they are
there to refuse.

Five things the virtual arm found that reading would not have:

1. **Upstream's collision meshes are its visual meshes.** As shipped the
   simulation runs at about an eighth of real time, under the 0.3 floor
   the simulator tier enforces. The overlay swaps each for its bounding
   box. The jaws are too coarse to grasp with as boxes; the grasp lesson
   has to switch those two links back to meshes and measure the cost.
2. **Upstream's effort limit is 10 N·m on every joint**, about five times
   an STS3215's stall torque. The overlay derives limits from the
   datasheet, as the 3VN arm does. The arm still holds every named pose
   at 1.30 N·m.
3. **`move_joint` drove the gripper into the bench.** Its jogs
   accumulated, which one arm's hand-picked targets had hidden. Each
   joint now returns before the next moves. Mock passed the old version;
   only physics caught it.

4. **A frictionless jaw ran into its hard stop and stayed there.**
   Closing with the gripper pointing down, gravity held the jaw at its
   velocity limit, and DART does not drive a joint that is on a limit.
   `pick_and_place` passed anyway: its release step checked nothing. The
   overlay now gives servo joints damping and friction (the 3VN arm's
   values, an estimate), the release step has a check, and with the
   friction removed that check fails. A joint commanded exactly onto a
   hard stop can still stick; the gripper's named travel and the teleop
   stay clear of the stops.

   This is worth a look on the 3VN arm. Its open grasp problem includes
   a gripper that "stayed fully open at 18 mm", and 18 mm is its upper
   limit. A finger pushed onto its stop by the cube would behave exactly
   like this. Not tested there.

5. **The 3VN arm's hardware interface never reaches the controllers.**
   Found while writing the SO-101's, by asking the same question of the
   ESP32 one. `Esp32SystemInterface` keeps positions and commands in its
   own vectors and never passes them to the framework's handles. Driven
   against a fake device: the device reports 0.4 rad and the state
   handle reads NaN; a controller commands 0.9 rad and the frame sent to
   the device carries 0.4, its "hold where you are" fallback. On the
   physical 3VN arm, `/joint_states` would be empty of meaning and the
   arm would never move. Its 14 unit tests pass because they count
   frames and never go through a handle. **Not fixed here**: it is the
   3VN arm's code and a change of its own. The SO-101 interface's tests
   read and write through the handles for exactly this reason.

Not done, and not claimed:

- **No real servo has answered any of this.** The hardware interface
  runs against a simulated bus written from the same reading of the
  protocol, so the two agree by construction. Register addresses, byte
  order, and each joint's zero and direction are assumptions until R1
  and R2.
- The servo figures are datasheet values that sellers disagree on, and
  the 5 V supply derating is an estimate. Both are marked in the profile.
- Nothing here runs in CI. `make test` skips the tests that need the
  fetched description, and says so; the two integration tiers are still
  run by hand.
- `make play` is wired to the mobile manipulator, and `make drive`
  needs a terminal: neither was run against the SO-101. The teleop's
  gripper keys were changed to use the gripper's named travel, untested
  beyond its unit tests.
- A joint commanded exactly onto a hard stop in Gazebo can stay there.
  Seen on the jaw's upper limit. The likely fix is to put the
  simulator's stops a little outside the command limits.

## 2. The 48 lessons on a virtual SO-101

**Runs** means the lesson's commands work today with
`PROFILE=so101_follower` added to the launch command, and its text does
not depend on which robot it is. **Version** means the commands work or
nearly work, but the text, a joint name or a number is the 3VN arm's.
**New** means the SO-101 needs a lesson that does not exist. **Not on
this path** means it is about the 3VN arm's own hardware, or is parked.

"Checked" is what was actually run on the SO-101 in this work, not what
is expected to work.

### Stage 1: Your environment

| # | Lesson | On the SO-101 | Checked |
|---|---|---|---|
| 1 | Docker, and nothing else | Runs | robot-agnostic |
| 2 | Living inside the container | Runs | robot-agnostic |
| 3 | A test suite people actually run | Runs | `make test` |
| 4 | The decision everything else rests on | Version: `TARGET=feetech` and `FeetechSystemInterface` in place of `esp32`. The plugin now exists. | the four `make urdf` greps |
| 5 | Before the first line, a licence and a name | Runs. It is the SO-101's licence story. | text only |
| 6 | Symptoms that lie | Runs | robot-agnostic |

### Stage 2: Describing a robot

| # | Lesson | On the SO-101 | Checked |
|---|---|---|---|
| 7 | A robot is a tree | Version: five revolute joints and one jaw, no mimic, root is `world` | `make urdf`; the `make view` launch, in RViz |
| 8 | One source of physical truth | Version: the profile is an overlay; upstream's URDF is the kinematic truth | profile tests |
| 9 | Never type a number you can derive | Version: the inertias are upstream's, exported from CAD. The lesson becomes "check a number you did not derive". | no |
| 10 | Three frames that save you a day each | Version: `gripper_frame_link`; there is no camera frame until a camera is added | TF in `test-ros` |
| 11 | One file, three targets, two traps | Not on this path: xacro detail of the 3VN arm | |
| 12 | Test it with two parsers, not one | Version: `check_description.sh` globs `threevn_*.yaml` and skips the SO-101 | `check_urdf` by hand, all three targets |
| 13 | Limits come from the datasheet | Version, and a better one: upstream ships placeholders and the overlay replaces them | profile tests |

### Stage 3: Making it move

| # | Lesson | On the SO-101 | Checked |
|---|---|---|---|
| 14 | A simulator that runs in CI | Version: `make play` is wired to the mobile manipulator. `make sim` runs, without RViz. | `make sim` |
| 15 | Three ways to watch | Version: depends on `make play` | no |
| 16 | Drive it yourself | Version: the teleop reads its joints from the controller, so it should work against `make sim`. Needs a terminal; not run. | no |
| 17 | Topics, services and actions | Runs, except the `/base_controller` step | graph checks in `test-ros` |
| 18 | Two clocks, and the bug they cause | Runs | `/clock` in `test-sim` |
| 19 | One command, two backends | Version: the goal in the lesson names the 3VN arm's joints | the same goal with SO-101 names, in `test-ros` |
| 20 | Scenarios are tests you can watch | Runs | 6 of 6 on mock, 6 of 6 in Gazebo |

### Stage 4: Make it trustworthy

| # | Lesson | On the SO-101 | Checked |
|---|---|---|---|
| 21 | Four layers of test | Runs | all four: the acceptance run passed 6 of 6 in Gazebo |
| 22 | Tests that cannot pass by accident | Runs | robot-agnostic |
| 23 | Keeping the suite fast | Runs | robot-agnostic |
| 24 | Integration tests without physics | Runs | `make test-ros PROFILE=so101_follower` |
| 25 | An image that carries no simulator | Version: the runtime image has no `vendor/`. It needs the pinned fetch as a build step. | no |
| 26 | CI that runs what you run | Runs. The SO-101 is not in the pipeline yet. | no |
| 27 | A release is a tag plus evidence | Runs | robot-agnostic |
| 28 | A pipeline for a machine with motors | Runs as a concept; its firmware half is the 3VN arm's | |

### Stage 5: Make it real

| # | Lesson | On the SO-101 | Checked |
|---|---|---|---|
| 29 | The microcontroller is a dumb driver | New lesson; the code exists. The servo is the driver, and the interface is a third the size of the ESP32's. | the interface's 23 unit tests |
| 30 | A wire protocol you can test on your laptop | New lesson; the code exists. Somebody else's protocol, checked against packets worked out by hand, then against a simulated bus. | 24 protocol tests, 16 on the simulated bus |
| 31 | Decide how it fails before it fails | New lesson; half the code exists: a missing servo, a quiet servo, a corrupt reply and torque-on are decided and tested. What a servo does when the HOST goes quiet is not. | the Link and Activate tests |
| 32 | Where safety actually lives | Version: joint names in the hand-sent goal | `safety_limit`, and the clamp test in `test-ros` |
| 33 | Parts generated from the same numbers | Not on this path: the SO-101's parts are upstream's STLs | |
| 34 | A decision with a number attached | Not on this path | |
| 35 | Deploy, and roll back with a pull | Version: needs lesson 25's image | no |
| 36 | Alive is not the same as ready | Version, and a better one: the servo reports load and temperature. See R5. | the dashboard against the SO-101 in Gazebo: `/readyz` ready, six joints in `/api/state` |
| 37 | Metrics from a robot behind a home router | Runs | robot-agnostic |

### Stage 6: Make it capable

| # | Lesson | On the SO-101 | Checked |
|---|---|---|---|
| 38–40 | Mobile base, gyro, composition | Not on this path: parked with the mobility work | |
| 41 | Solve the kinematics | New: numerical IK. The closed form refuses five joints, correctly. | the refusal, in the spike |
| 42 | Prove the grasp | New: needs lesson 41 and mesh collisions on the jaws | no |
| 43 | A camera, and the geometry behind it | New: no camera in this description. Upstream added a wrist-camera URDF variant in September 2026. | no |
| 44–47 | Datasets, model, registry, serving | Version: the registry is robot-agnostic; collection is wired to the mobile manipulator's camera | no |
| 48 | Somebody else's robot | Runs, updated in this change: the last step now passes | all of it |

**Count:** 17 run, 19 need a version, 6 need a new lesson, 6 are not on
this path. The free stages (1 to 3) are 8 runs and 11 versions with no
new lessons, so a free SO-101 track is a writing job, not a building
one, apart from `make play`.

## 3. What to build next on the virtual arm

In the order that unblocks the most lessons:

1. ~~A fake servo bus, and the `feetech` hardware interface against
   it.~~ **Done.** All six scenarios pass through the real plugin
   against the simulated bus, and the integration tier runs with
   `TARGET=feetech`. Lessons 29 to 31 now have code and tests to be
   written around.
2. **A transport the Mac can reach.** Half done: the container's end
   (the TCP transport) exists and is what the simulated bus is reached
   through. The host's end, a serial-to-TCP bridge for the real adapter
   on macOS, does not. It has to exist before R1 if R1 is recorded on
   the Mac. The 3VN arm's interface still refuses `tcp`.
3. **`make play` for an arm-only profile**: simulator plus RViz, so
   lessons 14 to 16 run.
4. **The SO-101 tiers in CI**: fetch the pinned description, run
   `test-ros` and `test-sim` for both profiles. `make test-ros` is not
   in CI for either robot today.
5. **Numerical IK**, against the existing IK test cases. Unblocks
   lessons 41 and 42.
6. **The runtime image with the pinned description baked in.**

## 4. Sessions to record on the physical arm

Each session has the same shape, so that the swap is the event and
everything around it is rehearsed:

- **Rehearse:** the exact commands, run on the virtual SO-101 first. If
  they do not pass there, the session is not ready to film.
- **Swap:** the single thing that changes when the real arm takes over.
- **Evidence:** what the session leaves in the repository.
- **Stop if:** the condition that ends the take.

Standing rules for every take: the arm clamped to the bench, a hand on
the power switch, the first motion of any session slow and small, and
the camera kept rolling through a failure. On this course the failure is
usually the lesson.

### R1. Six servos, one bus

Before assembly, because a servo can only be given its id while it is
alone on the bus.

- **Rehearse:** the simulated bus already renumbers a servo when its id
  register is written, as a real one does. What is missing is the small
  tool that does the writing: set an id, ping it, read it back. Build
  and test it against `make fake-bus` first.
- **On camera:** the adapter board, power, one servo at a time. Set ids
  1 to 6 in the order of `joints:` in the profile. Read back position,
  voltage and temperature from each.
- **Swap:** none. This is the first contact.
- **Evidence:** a table of six ids with their replies, and the supply
  voltage the servos report. That number settles whether
  `limit_derate.effort` (0.68, an estimate for a 5 V supply) is even the
  right question. This is also the first time the register table in
  `feetech_protocol.hpp` meets a servo: a wrong address shows up here
  as a voltage of 200 V or a temperature of zero.
- **Stop if:** a servo does not answer at its id, or two answer at one.

### R2. Assemble, and find zero

- **Rehearse:** `make view PROFILE=so101_follower`, joints at zero. That
  picture is the pose the real arm is put in to be zeroed: upper arm
  vertical, forearm and gripper level. (Checked: RViz shows upstream's
  meshes, with six sliders.)
- **On camera:** assembly (timelapse), then calibration: the arm held in
  the zero pose and each encoder's offset read.
- **Swap:** none yet. The offsets are what will make the swap honest.
- **Evidence:** `zero_count` and `direction` for each of the six joints
  in the profile, changed from `assumed` to `measured`. They are 2048
  and +1 everywhere today.
- **Stop if:** a joint cannot reach both of upstream's limits by hand
  with power off. That is an assembly fault, not a software one.

### R3. One command, three backends

The central recording. The same scenario on mock, in Gazebo and on the
arm, with one argument changed.

- **Rehearse:**
  ```bash
  make mock  PROFILE=so101_follower            &&  make scenario NAME=home
  make sim   PROFILE=so101_follower            &&  make scenario NAME=all
  make robot PROFILE=so101_follower BUS=fake   &&  make scenario NAME=all
  ```
  All three pass today. The third is the real hardware interface with
  six simulated servos behind it.
- **Swap:** drop `BUS=fake`. Same target, same plugin, same launch
  file, same controllers file, same topics and actions, same six
  scenarios. Nothing above the transport is edited on camera.
- **On camera:** `home`, then `ready`, at three times the usual
  duration. Then `make scenario NAME=all`.
- **Evidence:** the scenario report from the real arm, committed beside
  the two from the virtual one.
- **Stop if:** the first `home` moves any joint the wrong way. That is a
  direction sign from R2, and it is fixed in the profile, not in code.
- **Before the take ends:** drive to `rest`. Stopping the stack releases
  the servos, and the arm goes limp where it is.
- **Needs first:** R1 and R2, and on a Mac the host-side bridge.

### R4. What the datasheet promised

- **Rehearse:** record `make scenario NAME=move_to_position` in Gazebo
  with the joint states logged. That is the prediction.
- **Swap:** the same scenario on the arm, logged the same way.
- **On camera:** the two traces overlaid. Then no-load speed and holding
  load, read from the servos themselves.
- **Evidence:** `servos.sts3215`, `limit_derate` and the joint friction
  in `defaults` move from `datasheet` and `estimated` to `measured`, and
  the simulated arm is re-run with the measured values.
- **Stop if:** a servo reports a temperature climbing while holding
  `ready`. Go to `rest` and power down.

### R5. A servo that tells the truth

- **Rehearse:** in Gazebo, a block placed in the arm's path.
- **Swap:** a hand on the real arm.
- **On camera:** hold a joint still during a move. The load reading
  rises and the readiness check says so. On the 3VN arm nothing would
  notice.
- **Evidence:** the readiness check, with a test, and one recorded
  trace.

R1 to R3 are "the first few". R4 and R5 follow as soon as R3 passes.

## 5. Decisions that are yours

1. ~~Adopt a community Feetech plugin, or write one.~~ Written, on
   your go-ahead of 2026-11-15. The profile names the plugin, so the
   community one remains a one-line alternative.
2. **Where the real arm plugs in.** A Linux host reaches USB directly;
   the Mac needs the TCP bridge. If the sessions are recorded on the
   Mac, the bridge has to exist before R1.
3. **Whether the SO-101 tiers go into CI now.** They cannot be verified
   without a push, so they are not in this change.
4. **Which supply the servos get.** The profile assumes the official
   5 V. If the order was for 7.4 V, `limit_derate` changes.
5. **Whether the arm goes limp on stop.** Deactivating the hardware
   releases the servos, as the 3VN arm's interface does. The
   alternative is to hold the last pose with nothing controlling it.
   Released is the default because it is the safer failure for a person
   near the arm, and the less safe one for the arm.
6. **Cross-checking the simulated bus against Feetech's own SDK.** It
   would be an independent reading of the protocol, before any servo
   arrives. It means installing a third-party package in the dev
   container, so it has not been done.

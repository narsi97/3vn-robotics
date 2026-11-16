# Pilot recording: one command, three backends

The first recording, and a test of the recording process itself. It
needs no hardware: the SO-101 runs as a virtual arm throughout.

| | |
|---|---|
| Length | 12 to 15 minutes finished |
| Robot | SO-101 follower, virtual (`PROFILE=so101_follower`) |
| Needs | Docker, this repository on `main`, `make setup` already done |
| Recorded in | English. Dubbed and subtitled afterwards; see [terms.yaml](terms.yaml) |

## What the viewer leaves with

One sentence: **the same scenario passes on three different things
underneath, and only one line of the robot's description changed.**

Everything in the session serves that sentence. If a segment does not,
cut it.

## What this pilot is for

It is the smallest session that goes through the whole process once, so
the process can be judged before twenty hours are recorded with it:

1. Does the dubbed voice sound right to a native speaker?
2. Do the terms in `terms.yaml` survive translation, in speech and in
   subtitles?
3. Do the subtitles stay in step where the dub runs longer than the
   English?
4. What does one finished minute cost, in money and in your time, per
   language?

## Before recording

**The machine**

- Nothing else building or testing. The simulator steps are sensitive to
  load: with a second container working on this Mac, the Gazebo tier
  missed its timing floors, and passed when run alone.
- `git switch main && git pull`, then `make doctor`.
- `make so101-fetch` has been run once before, so segment 1 is a re-run
  and takes about four seconds.
- `make stop` to start from nothing running.

**The picture**

- 1920×1080, 30 fps. Notifications off.
- Left half: four terminal tabs, named `robot`, `view`, `client` and
  `bus`. Font 18 pt or larger, a short prompt.
- Right half: a browser at <http://localhost:8106/vnc.html>. This is
  where the arm appears.
- **No text on screen in any language but English, and no burned-in
  captions.** Every language gets its own audio track and its own
  subtitle track over the same picture. Text baked into the picture
  cannot be translated.

**The voice**

- Short sentences. One idea each. They are easier to dub and they
  subtitle onto one line.
- No idioms and no jokes that depend on English.
- Name the thing instead of saying "this" or "that". A dubbed sentence
  loses the pointing finger.
- Do not read commands aloud. Say what the command does; the command is
  on screen.
- Leave two seconds of silence before each command and hold each result
  for three seconds. Dubs in Indian languages often run longer than the
  English, and the silence is where the extra length goes.

**Seeing the arm**

`make watch` opens RViz on whichever robot is running. It is the same
picture on all three backends, which is the point of the session.

- Start it in the `view` tab after each robot is up. It holds that tab
  while it runs, and takes about half a minute to draw the arm. Cut the
  wait.
- `make stop` closes it along with the robot, so it is started three
  times in this session.
- It draws at 6 to 10 frames a second, because the container renders on
  the CPU. The motion is slow enough that this reads as smooth.

## The session

`[face]` is you on camera, or an avatar. `[screen]` is screen capture
with your voice over it. Times are finished length, after cuts.

### 0. The promise `[face]` 0:30

> This arm is called the SO-101. I did not design it, and today I do not
> even have one.
>
> In the next fifteen minutes it will pass the same test three times. On
> nothing at all. In a physics simulator. And through the real driver
> for its motors.
>
> Between those three, exactly one line changes. I will show you the
> line.

### 1. A robot somebody else described `[screen]` 1:30

Tab `client`:

```bash
make so101-fetch
```

Shows: `fetching Simulation/SO101/so101_new_calib.urdf @ 5f6d2b876a53`,
thirteen mesh files, then `13 meshes in …/vendor/so101/assets`.

> The description of this arm is not in our repository. Another team
> wrote it, under a licence that lets us use it.
>
> We fetch it at one exact commit, and we check its checksum. A robot
> description is a dependency. A dependency gets a version.

### 2. Backend one: nothing `[screen]` 2:30

Tab `robot`:

```bash
make mock PROFILE=so101_follower
```

Ready when three lines say `Configured and activated`. About half a
minute. **Cut the wait.**

Tab `view`:

```bash
make watch
```

The arm appears in the browser: upper arm vertical, forearm and gripper
level. Leave it running.

Tab `client`:

```bash
make scenario NAME=home
```

Shows:

```
scenario: home
  ok   move to home                            2.66s
  PASS in 2.66s

  1/1 scenarios passed
```

> Five joints and a gripper. This picture is drawn from their file. We
> changed nothing in it.
>
> There is no robot here and no simulator. The mock target answers every
> command with "done".
>
> It sounds useless. It is the fastest way to find out that the software
> above the robot is wired correctly. It runs in seconds, so it runs on
> every change.

Then: `make stop`

### 3. Backend two: physics `[screen]` 4:00

Tab `robot`:

```bash
make sim PROFILE=so101_follower
```

Ready after about thirty seconds. **Cut the wait.** Then `make watch`
again in the `view` tab, and in `client`:

```bash
make scenario NAME=all
```

Six scenarios in about seventy-five seconds, ending
`6/6 scenarios passed`. The arm moves in the browser throughout. Play
`move_joint` (twenty-six seconds) at four times speed.

> Now the arm has weight. Gravity pulls on it and the motors have a
> limit.
>
> These are the same six scenarios, and nobody edited them. The first
> time I ran them here, two things went wrong that the mock target had
> passed.
>
> One scenario pushed the gripper into the bench. And the gripper ran
> into its own end stop and never opened again, while the test still
> said "pass".
>
> Both were real mistakes. Only physics found them. That is what a
> simulator is for.

Then: `make stop`

### 4. Backend three: the real driver, and no motors `[screen]` 4:00

Tab `bus`:

```bash
make fake-bus
```

Shows: `fake Feetech bus: 6 servos (ids 1,2,3,4,5,6) on 127.0.0.1:6560`

Tab `robot`:

```bash
make robot PROFILE=so101_follower BUS=fake
```

Shows, among the start-up lines:

```
[FeetechSystemInterface]: 6 servos answering on 127.0.0.1:6560 (tcp)
[FeetechSystemInterface]: torque enabled; holding the pose the arm was found in
```

and tab `bus` prints `fake Feetech bus: host connected`. Ready after
about thirty seconds. Then `make watch` again in the `view` tab, and in
`client`:

```bash
make scenario NAME=all
```

Six scenarios in about seventy seconds, ending `6/6 scenarios passed`.

> This is the code that will drive the real motors. The same plugin, the
> same protocol, byte for byte.
>
> What it is talking to is a program that pretends to be six motors. I
> wrote it because my motors are still in the post.
>
> It is not proof that the real motors will agree. It is proof that
> everything on our side is ready for the day they arrive.

Then: `make stop`

### 5. The one line `[screen]` 2:00

Tab `client`, one after another:

```bash
make urdf PROFILE=so101_follower TARGET=mock 2>/dev/null | grep "<plugin>"
make urdf PROFILE=so101_follower TARGET=gz 2>/dev/null | grep "<plugin>"
make urdf PROFILE=so101_follower TARGET=feetech 2>/dev/null | grep "<plugin>"
```

Shows, one line each:

```
      <plugin>mock_components/GenericSystem</plugin>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      <plugin>threevn_hardware/FeetechSystemInterface</plugin>
```

> Three backends. This is the line that changed.
>
> Everything above it stayed the same: the controllers, the scenarios,
> the tests. That is the whole design, and the rest of the course is
> built on it.

### 6. What this did not prove `[face]` 1:00

> No real motor has answered this code yet. When the motors arrive, the
> first thing I will do is ask each one a single question, and see
> whether it answers.
>
> That is the next recording. If it fails on camera, I will leave it in.

## After recording

1. Export the English transcript with timings, as
   `pilot-one-command-three-backends.en.srt`, beside this file. It is
   the source every translation is made from, and it is reviewed like
   code.
2. Translate it with `terms.yaml` applied, one file per language:
   `….hi.srt`, `….te.srt`, and so on, using the two-letter language
   code.
3. A native speaker reads each translation before any audio is made
   from it.
4. Dub from the reviewed translation. Publish each language as an extra
   audio track and subtitle track on the same video.

Steps 2 to 4 are not automated yet.

## What was checked

On 2026-11-16, on `main` at `0d6bde5` plus the `make watch` target added
with this file:

- Every command above was run as written, each through a real terminal,
  and the outputs and timings quoted are from those runs.
- `make watch` was run over all three backends with the SO-101, with a
  scenario moving the arm each time, and over the mock target with the
  3VN arm.
- With RViz attached to the simulator the real-time factor stayed
  between 0.77 and 0.99, above the 0.3 floor.

Not checked: the session has not been run end to end in one sitting, and
nothing has been recorded, transcribed or dubbed.

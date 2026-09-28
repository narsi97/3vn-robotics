# Learning this without a robot

The honest summary: **fourteen of the fifteen phases are completable with
Docker and nothing else.** That is not a concession made for students who
cannot afford hardware — it is the reason the architecture looks the way
it does. Hardware is one of three execution targets behind a
`ros2_control` seam, so "no hardware" is a target selection, not a
degraded mode.

This page is the path through it. It assumes you can write software and
have never touched a robot.

## Before anything else

```bash
make doctor && make setup
```

`doctor` checks the one thing that catches most people: Docker Desktop
ships with less memory than Gazebo needs. Set **Settings → Resources →
Memory to 6 GB or more** or Gazebo is OOM-killed by the VM and the
symptom reads like a Gazebo bug.

## Day one: see it move

```bash
make play
```

Headless Gazebo plus RViz, at **http://localhost:8106/vnc.html**. You
get the mobile manipulator on a grid, the camera feed it is producing,
and the trail its wheels think it left.

Then, in a **second terminal**:

```bash
make drive
```

```
  base        w  forward     s  back      a  left      d  right
              space  stop now
  arm         1..9  select joint      ,  jog down      .  jog up
              tab   next joint        h  home the arm
  gripper     [  open        ]  close
  speed       - _  slower    + =  faster
```

Drive it into the bench. Jog a joint into its limit and watch the teleop
refuse. That is the whole point of day one: the robot is a thing you can
push around, not a diagram.

### Three things to notice while driving

They are properties of the robot, and each one is a lesson:

1. **Let go of `w` and it stops.** Not because teleop sent a stop, but
   because `diff_drive_controller` brakes when it has not heard a command
   recently. Velocity is a *subscription with a watchdog*, not a setting.
   Kill the teleop mid-drive and the robot still stops.
2. **Jogging clamps at the joint limit** and says so. The limits come
   from `/robot_description` — the URDF the robot is actually running,
   not a copy in the teleop. Change the YAML, relaunch, and the clamp
   moves.
3. **It feels slow.** It is. See below.

## How fast this actually runs

**Roughly a third of real time**, on an M1 with 8 GB given to Docker.
So three wall-clock seconds of holding `w` moves the robot about as far
as one second would on a real machine. **Nothing is broken.** There is
no GPU in the container; physics and rendering are both on the CPU, and
`llvmpipe` is doing the work an Apple GPU would do.

Do not read precision into that number. Five consecutive 8-second
samples of the *same* run, with nothing changed between them, measured:

```
0.373  0.269  0.428  0.284  0.410      mean 0.353
```

That is a ±20% spread from background load alone, and across separate
runs the same configuration has measured as low as 0.22 and as high as
0.43. A single real-time-factor reading cannot tell you whether a
change helped — which is worth knowing before you try to optimise
anything here by measuring it once. RViz reports 15–24 fps over the
same range.

Two consequences worth internalising early:

- **Never time anything with `time.sleep`.** Sim time and wall time are
  different clocks running at different rates. Everything in this repo
  uses `/clock`, which is why `/clock` is the one non-optional bridge.
- **The Gazebo GUI costs about half your simulation rate** and shows you
  less than RViz does. `make play` keeps Gazebo headless for that reason.
  `make mm-sim GUI=1` exists when you specifically want Gazebo's render.

## What each phase gives you with no hardware

| Phase | What you do without a robot | Complete? |
|---|---|---|
| 0 Environment | Build the image, `make doctor`, read the ADRs | yes |
| 1 Description | Edit YAML, watch the URDF and the tests change | yes |
| 2 Simulation | `make sim`, `make scenario NAME=all` | yes |
| 3 Control | `make mock` vs `make sim` — same commands, two backends | yes |
| 4 Testing | `make test`, `make test-ros`, `make test-sim` | yes |
| 5 Firmware | Protocol, framing and `Esp32SystemInterface` are written and unit-tested against a fake transport | **software only** |
| 6 Docker | `make runtime`, `make runtime-verify` | yes |
| 7 CI/CD | Fork it; the workflows run on your own Actions minutes | yes |
| 8 Deployment | Everything but pushing to a VPS you own | mostly |
| 9 Monitoring | Traces and metrics from the simulated robot | yes |
| 10 Mobile base | `make base-sim`, `make base-drive`, `make heading` | yes |
| 11 Mobile manipulator | `make play`, `make mm-verify` | yes |
| 12 Perception | `make perception`, `make perception-verify` | yes |
| 13 Data | `make dataset`, `make dataset-describe` | yes |
| 14 ML | `make train` — a real model on data you generated | yes |
| 15 MLOps | `make registry-*`, `make serve` | yes |

Phase 5 is the only genuine gap, and it is narrower than it sounds: the
firmware, the wire protocol and the hardware interface all exist and are
tested. What you cannot do is watch a real servo fail to reach a
commanded angle.

## What simulation will not teach you

Being straight about this matters more than the list above, because
every one of these has already cost this project something real:

- **Mass and inertia.** Until Phase 15's CAD work, every link mass in
  this repo was a guess. The base was 2.3× too heavy and the shoulder 40%
  too light. Tipping margins had been computed for fifteen phases against
  a robot nobody had designed. Simulation will happily run a lie.
- **Backlash, friction and flex.** Gazebo joints are perfect. A printed
  PLA arm on hobby servos is not, and the gap shows up as an end effector
  that is tens of millimetres from where the URDF says it is.
- **Press fits and tolerances.** No geometric check tells you whether a
  bushing presses into its bore. You print one and try it.
- **Power.** Nothing here browns out, stalls or overheats.
- **Skid-steer scrub**, partially. It is modelled, but the coefficient
  depends on your actual wheels and your actual floor.

The correct way to hold this: **simulation tells you whether your
software is right. It does not tell you whether your robot is right.**
That is exactly why the seam exists — so the software can be finished and
trusted before any of the above is known.

## A route through it, in order

1. `make play` and `make drive`. Break things. Drive into the bench.
2. `make test` — read three tests, not thirty. Start with
   `test_ros2_control_targets.py`: it is the architecture, asserted.
3. Change a number in `config/threevn_arm_v1.yaml` — a joint limit, a
   link length. Run `make test`. Watch which tests fail and why.
4. `make mock`, then `make sim`, and send the *same* action goal to both.
   That is the decoupling claim, demonstrated rather than asserted.
5. `make scenario NAME=pick_and_place` with `make play` running.
6. `make dataset` then `make train`. The model is small on purpose; the
   pipeline around it is the lesson.
7. Fork the repo and let CI run. Break a test deliberately and watch it
   go red.

## When you do get hardware

Nothing above is thrown away. You change one argument:

```bash
make robot            # target=esp32 instead of target=gz
```

Today that fails with a pluginlib "class not found" until the firmware is
flashed, which is the honest state of the project. Every scenario, every
test and every line of `RobotClient` stays exactly as it is — because
none of them ever knew which target they were talking to.

See [bom-hardware.md](bom-hardware.md) for what to buy and
[cad.md](cad.md) for what to print.

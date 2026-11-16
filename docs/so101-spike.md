# Spike: running the 3VN stack against somebody else's robot

```bash
make so101-fetch     # once: pulls the Apache-2.0 description into vendor/
make so101           # spawns it in Gazebo under OUR controllers
```

> This page records the spike as it was run. Its one failure has since
> been fixed, and the spike's own launch file is gone: the SO-101 is now
> a profile, described on [its own page](so101.md). What is below is why.

## The question

The architecture's central claim is that the application does not know
what it is driving. Every test of that so far swapped the execution
*target* — mock, Gazebo, ESP32 — under one robot we designed. That is
the easy half. This swaps the **robot**.

The arm is TheRobotStudio's SO-101: 5 DOF plus a jaw, STS3215 bus servos,
Apache-2.0. Nothing of ours describes it, and it was deliberately not
re-authored into a 3VN profile — that would have tested our xacro rather
than our stack. The upstream URDF is used as it ships. The launch file
adds only what it lacks for Gazebo: a `<ros2_control>` block naming its
joints with the same `GazeboSimSystem` plugin our own robot uses, the
`gz_ros2_control` plugin tag, and a `world` anchor, because upstream has
no `world` link and a free 632 g arm topples the moment a joint moves.

## What happened

It spawned, and all three controllers came up active with no errors.

| | |
|---|---|
| `connect()` | ok |
| joint discovery | ok — found `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll` unaided |
| `/joint_states` | ok — 6 joints |
| `get_arm_pose()` | ok — 5 values |
| `get_limits()` | ok — ±110° on the pan, read from its URDF |
| TF to the tool frame | ok |
| **`move_joints()`** | **ok — the arm moved** |
| **`open_gripper()`** | **ok — its jaw opened** |
| `move_to_named('home')` | **FAIL** — `expected 5 joint positions, got 4` |
| `arm_geometry()` | **refused**, correctly |

## The one thing that broke

`POSES` in `robot_client.py` is a module-level dict of four-element
lists. Joint *names* are discovered from the controller, so the client
generalises — but the named poses those joints are commanded to do not.
Every scenario starts with `home()`, so every scenario stops there.

That is a single hardcoded constant standing between this stack and a
robot nobody here designed. The fix is to move named poses into the
profile, where every other physical fact about a robot already lives.

## The refusal is the other result

`arm_geometry()` would not solve the SO-101, and said exactly why:

```
gripper_frame_link -> closed-form IK needs exactly 4 driven joints ...; this chain has 5
wrist_link         -> 'shoulder_lift' must turn about Y for this closed form
```

This is the structural guard behaving as designed. A closed form applied
to an arm it does not fit does not fail — it returns joint angles that
are wrong. Refusing with a true reason, on a real foreign arm, is worth
more than the guard passing on our own.

## What the SO-101 would fix

Measured from its URDF, against the limits that made our arm hard to
grasp with:

| joint | 3VN arm | SO-101 |
|---|---|---|
| shoulder_pan | ±90° | ±110° |
| shoulder_lift | −15…+120° | ±100° |
| **elbow** | **−120…+10°** | **±96.8°** |
| wrist_flex | ±90° | ±95° |
| wrist_roll | — | −157…+163° |

Our elbow bends essentially one way, which is precisely why top-down
grasping is unavailable on our arm above its own base. The SO-101's is symmetric,
and it adds a wrist roll we do not have. Its links are longer too — upper
arm ~116 mm and forearm ~135 mm against our 110/95 — so the 250–270 mm
annulus that is our only table-height reach should open up.

None of that is verified here. This spike tested the software seam, not
the arm's workspace.

## What happened next

The constant moved. Named poses are in each robot's profile, the
description carries them, and the client reads them from the robot it
is attached to. `make scenario NAME=all` passes on the SO-101, on mock
and in Gazebo.

Making that true found two more things this spike could not, because it
stopped at `home()`: upstream's collision meshes run the simulator at an
eighth of real time, and one scenario drove the longer arm into the
bench. Both are on [the SO-101 page](so101.md).

## What this does not say

It does not say to adopt the SO-101. It says the seam holds, the licence
permits it, and the cost of trying it is one constant and a profile. The
$122 price against our ~$30 arm is a product decision, and
`docs/bom-hardware.md` would have to change honestly if it were taken.

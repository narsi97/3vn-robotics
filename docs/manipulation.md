# Manipulation: what this arm can actually do

```bash
make workspace            # straight down (180 deg)
make workspace PITCH=90   # side-on
```

Written after adding inverse kinematics, because solving the IK is what
made the arm's real limits measurable instead of assumed.

## The solver

`kinematics.py` gained `inverse_kinematics`. The arm is a turntable
carrying a planar 3R arm — `shoulder_pan` about Z, then `shoulder_lift`,
`elbow` and `wrist` all about parallel Y axes, every link offset purely
along +Z at zero. That structure has an **exact closed form**: yaw comes
straight from `atan2(y, x)`, the last link is subtracted using the
requested tool pitch, and what remains is a two-link problem with a
cosine rule. No solver, no iteration, no convergence to worry about.

Geometry is **measured from the URDF**, not copied from the YAML —
`arm_geometry()` walks the chain and reads the offsets at the zero
configuration:

```
a1 (lift -> elbow)   110 mm
a2 (elbow -> wrist)   95 mm
a3 (wrist -> grasp)   70 mm
shoulder              65 mm above base_link
```

Change a link length and the IK follows, with no second place to edit.
A test asserts the long-reach profile measures differently.

**It refuses arms it does not fit.** A closed form applied to the wrong
structure does not fail — it returns confident nonsense. So
`arm_geometry` checks the joint count, the axis directions and the
along-Z offsets, and raises rather than solving.

### Verified against the running robot

Commanded through `RobotClient`, measured through TF — not compared
against the same arithmetic that produced it:

| commanded (mm) | measured (mm) | error |
|---|---|---|
| (260, 0, 30) | (260, 2, 33) | 3.3 mm |
| (260, 50, 30) | (260, 49, 30) | 0.7 mm |
| (250, −50, 50) | (250, −50, 50) | 0.4 mm |
| (190, 0, 200) | (191, 0, 200) | 0.7 mm |
| (200, 60, 170) | (200, 60, 170) | 0.3 mm |

The round trip through `forward_kinematics` agrees to 1e-6 m, so the
sub-millimetre-to-3 mm spread here is the **controller settling**, not
the solver.

## The finding that matters

**This arm cannot grasp from directly above anything it could reach on
the surface it stands on.**

Measured with `make workspace PITCH=<deg>`, re-run on 2026-10-06; heights
are relative to the bottom of the arm's base:

| approach pitch | share of reachable area | radius (mm) | height (mm) |
|---|---|---|---|
| 60° | 66% | 20–260 | +30 to +300 |
| 90° (side-on) | 42% | 70–270 | −10 to +270 |
| 120° | 19% | 200–260 | −40 to +180 |
| 150° | 5% | 210–230 | −50 to +110 |
| **180° (straight down)** | **1%** | **200 only** | **−50 to −40 only** |

`elbow_joint` is limited to [−120°, +10°], so it bends essentially one
way. The only straight-down poses left are a single 200 mm ring 40–50 mm
*below* the base — inside whatever the arm is standing on, where nothing
can be picked up. A test pins it from 20 mm below the base upward, so
that widening the limit later shows up as a failure rather than a silent
change of capability.

An earlier version of this table counted zero straight-down cells and
said "anywhere". Re-running it for the course showed that was not quite
true — the base has since grown 15 mm and the tool now reports shares
rather than cells — and the claim is now the one the measurement
supports.

And at table height — anything resting near the arm's own base plane —
the reachable band is only **250–270 mm radius**: near full extension,
approached from the side. Points reachable at several different pitches
exist only high up, around z = 170–220 mm.

### What this means for picking

- **Objects must sit in a narrow annulus**, roughly 250–270 mm from the
  yaw axis, and be approached side-on at about 90°.
- **There is no approach-angle freedom down there**, so any grasp
  planner that assumes it can choose an approach will be disappointed.
- Anything that sorts objects has to place them, and its bins, inside
  that band. This is the constraint to design around, and it is cheaper
  to know now than after building a perception stack that finds objects
  the arm cannot reach.

## The `pick` pose was never a grasp

Its pitch works out to 23° — the gripper pointing *upward* — and the
pose is fixed in joint space, derived from nothing about where the
object is. Measured against the cube in `bench_with_target`:

```
target cube (world)        x=+0.400  y=+0.000  z=+0.025
grasp frame at POSE 'pick' x=+0.079  y=+0.074  z=+0.369
```

450 mm apart. The `pick_and_place` scenario is choreography: it moves
through poses and opens and closes the gripper, and its `close gripper`
step has no post-condition, so nothing ever checked that an object was
held. **The robot has never picked anything up.**

That is not a bug in the scenario — it was honest about being a
sequencing demo — but it does mean a verified grasp is still to be
built, and that it is the next thing needed, not the perception.

## Verifying a grasp, and where it stands

```bash
make play WORLD=grasp_bench     # in one shell
make grasp                      # in another
```

The grasp is asked about **two ways, separately, plus a control**:

| check | from | survives on hardware |
|---|---|---|
| `holding()` | the gripper's own joint state: commanded closed, stopped short | yes — it is all a real robot will have |
| lifted | simulator ground truth: did the object rise more than 20 mm | no — and it cannot be fooled |
| control | the same grasp, at empty air | — |

Either check alone would have been wrong. The first `holding()` tested
only that the fingers had not fully closed, and passed a grasp that had
merely **shoved the cube 5 mm sideways**: a gripper that never closed
sits at its open limit, which is also "not fully closed". It now
requires the fingers to stop strictly *between* closed-on-air (1.7 mm)
and open.

### Things that had to be true first

- **A cube that fits.** `bench_with_target`'s 50 mm target never fitted
  a jaw that opens about 45.7 mm, and it is `<static>`, so contact could
  never have moved it. `grasp_bench` has a 35 mm, dynamic, 20 g cube.
- **Somewhere the arm is strong.** At floor level the arm is at full
  extension and the edge of its travel, where it sags 6° short and
  misses by 43 mm. The cube sits on a 140 mm pedestal instead, 230 mm
  out, where 37 approach angles work and the tightest joint keeps 31° of
  margin.
- **The jaws, not the frame.** The pinch point is 30 mm *behind*
  `grasp_frame`, measured through TF. Driving `grasp_frame` to the
  object puts the object past the fingertips. Because the offset lies
  along the approach direction, the pitch and the target are solved
  together, each candidate pitch tested against its own displaced point.
- **Room to approach.** At a fixed pitch the reachable set near the
  ground is a curve 4–6 mm wide, so the stand-off is directly above the
  object and each waypoint chooses its own approach angle.

### It does not close yet

**The grasp does not pick the cube up.** Measured on 2026-10-06:

```
  GRASP the cube
    target 230 mm out, +79 mm relative to the arm base
    fingers stopped at  0.99 mm  -> holding=False
    cube rose -0 mm            -> lifted=False

  CONTROL: grasp empty air
    target 230 mm out, +79 mm relative to the arm base
    fingers stopped at 18.00 mm  -> holding=False
    cube rose +0 mm            -> lifted=False
```

The jaws close to about a millimetre on a 35 mm cube that should be
between them, meeting no resistance, and the cube does not move.

Two things were wrong underneath this, found on 2026-11-18. The first
is fixed; the second is now a fact instead of a hypothesis.

**The gripper could not be relied on to move at all.** On the arm-only
robot (`make sim`) a gripper goal could come back `stalled: true,
reached_goal: false` with the finger at −5.4e‑21 m, and every goal after
it did the same. Not every time: on six other fresh spawns it worked.

The cause was in the profile. The finger's travel was its limits, 0 to
18 mm both, so it **spawned on its lower stop and was commanded onto
both stops**. Gazebo's physics engine, DART, gives each joint one
constraint per step, in the order position limit, velocity limit,
servo (`JointConstraint::update()`, DART 6.13.2). A joint at or past
its stop gets the stop, and the servo that `gz_ros2_control` drives a
position command through is skipped. The finger came to rest within
1e‑10 m of the stop, and rounding decided the side: 7e‑15 to 2e‑11 m
above it on the spawns measured here, and it moved; −5.4e‑21 m once,
and it never did.

Waiting for rounding cannot be tested, so the test pushes instead. The
closed finger is pressed toward its closed end with 0.05 N, a
twentieth of its 10 N effort limit, and told to open:

```
                       BEFORE: stops = travel        AFTER: stops 1 mm outside
  spawned              2.0e-11 m from its stop       1 mm from its stop
  closed, pressed      -2.5e-14 m: PAST its stop     -4.9e-12 m: 1 mm clear
  told to open         stalled: true                 stalled: false
                       reached_goal: false           reached 17.998 mm
                       never moved                   moving after 0.06 s
```

Without the press the old gripper usually worked, which is why it was
hard to see. With a 0.5 N press the finger stayed past its stop at
−2.5e‑13 m even after the press was taken away, and every open after
that stalled. That is the state the stalled run was in, reached from the
other direction.

The fix is in the profile and the description:

- each finger's `<limit>` is its **hard stop**, now 1 mm outside the
  travel: −1 to 19 mm of slide, 0 to 18 mm commanded. `open` and
  `closed` did not change;
- the gripper's `ros2_control` command range is its **travel**, not its
  limits. ros2_control enforces the tighter of the two, so a raw goal
  sent to a stop now stops 1 mm short of it.

A real servo wants the same gap: one driven into a hard stop is a
stalled servo. Three test tiers hold it: the travel and command range in
`make test` (`test_limits_match_yaml.py`, `test_describe.py`), the clamp
against a running controller in `make test-ros`, and the spawn position
and the press in `make test-sim` (`test_gz_gripper.py`).

One limit remains. A press above about **0.1 N** still beats the servo:
mass × velocity limit ÷ time step, 0.002 kg × 0.05 m/s ÷ 1 ms. Anything
that hard takes the finger to its velocity limit in one step, the
velocity-limit constraint replaces the servo, and the finger runs into
its stop at 0.05 m/s. Measured: 0.05 N held at 9 mm, 0.1 N ran it into
the stop. This is what gravity did to the SO-101's jaw
([so101.md](so101.md)). Arm motion is nowhere near it: 0.1 N is 50 m/s²
on the finger, which at the grasp's 230 mm reach would take a pan
acceleration over 200 rad/s². A contact force along the finger's axis
could exceed it.

**The right finger does not push.** The mimic hypothesis is confirmed.
At every spawn gz-sim prints:

```
Attempting to create a mimic constraint for joint [gripper_right_finger_joint]
but the chosen physics engine does not support mimic constraints, so no
constraint will be created.
```

Measured from Gazebo's own link poses, `/world/empty_bench/dynamic_pose/info`:

```
                              left finger    right finger   (world y)
  closed                      +10.000 mm     -10.000 mm
  left commanded to 18 mm     +28.000 mm     -10.000 mm    <- would be -28.000
```

RViz shows both jaws opening, because `robot_state_publisher` applies
the mimic itself. In physics the right finger never moves on its own; it
slides between its stops when something pushes it.

So the grasp is one finger pushing the cube against a finger that slides
away, which fits "close to about a millimetre, meeting no resistance".
The fallback, planned since the first design and tracked in
[risks.md](risks.md), is two independently driven fingers on one
controller, and it is the next thing to do. Not yet ruled out: that the
jaws are not around the cube when they close.

### The control did not use to run

Until this run, the control grasped a point 120 mm to the side of the
cube — 259 mm out, at a height where the arm reaches 230. The solver
refused it, the control never ran, and a refused control was counted as
"no false positive". Running the course's grasp lesson is what showed
it. The control now grasps the cube's own position swung 35° about the
arm's yaw axis — same radius, same height, so reachable whenever the
grasp is — and a control that cannot run is now a failure.

It runs now, and it found one more thing: on the control the fingers
stayed at **18 mm, fully open**. `holding()` said no, but because the
gripper never closed, not because it closed on nothing.

That was the hard-stop lockout above, from the other end: `open` was
then exactly the upper stop. Re-run on 2026-11-18 with the stops moved
out:

```
  GRASP the cube
    fingers stopped at  0.36 mm  -> holding=False
    cube rose -0 mm            -> lifted=False

  CONTROL: grasp empty air
    fingers stopped at  0.40 mm  -> holding=False
    cube rose +0 mm            -> lifted=False
```

The control now closes on empty air and `holding()` says no, which is
the case the control exists for, observed for the first time. The grasp
still does not lift the cube, which is what a passive right finger
predicts. Two independently driven fingers are the next thing to settle.
The verifier exists so that "it works" will be a measurement when it
does.

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

**This arm cannot grasp from directly above. Anywhere.**

| approach pitch | reachable cells | radius (mm) |
|---|---|---|
| 60–90° (side-on) | 94 → 59 | 15–270 |
| 120° | 26 | 210–255 |
| 150° | 5 | 225 |
| **180° (straight down)** | **0** | — |

`elbow_joint` is limited to [−120°, +10°], so it bends essentially one
way. No configuration both reaches a point and leaves the gripper
pointing straight down. A test pins this, so that widening the limit
later shows up as a failure rather than a silent change of capability.

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

## `POSES['pick']` was never a grasp

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

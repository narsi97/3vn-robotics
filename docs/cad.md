# Parametric CAD

```bash
make cad          # generate STL + STEP for both mechanisms
make cad-test     # geometry and printability checks
```

Output lands in `cad/out/<mechanism>/`, which is gitignored — generated
geometry is a build product, and committing it would create a second
copy of the truth that goes stale the moment the profile changes.

## Generated, not drawn

`threevn_arm_v1.yaml` already holds every length and the URDF reads it
directly. Drawing the parts in a GUI would make the printed part and the
simulated model two independent statements about one robot, and they
would disagree within a week — silently, because neither knows about the
other.

`test_the_upper_arm_length_comes_from_the_profile` asserts they agree,
and `test_changing_the_profile_changes_the_part` asserts the generator
actually reads its input: scaling the profile's link length by 1.5 must
scale the printed beam by 1.5. Without that second test, a generator
that ignored its input would pass everything else.

The profile gained two blocks for this: a **physical envelope** per
servo (case, flange, hole spacing, shaft offset — `provenance:
datasheet`) and a **`fabrication`** block holding nozzle, layer height,
wall thickness and hole clearance. Those describe the *printer*, not the
robot: the same arm on a better machine wants different clearances and
nothing about the kinematics changes.

## Mechanism is a parameter

The direct-drive and parallel-linkage arms are not two designs in two
files. They are one generator called with `mechanism='direct'` or
`'linkage'`, so the comparison is between two things built from the same
numbers rather than two things drawn on different days.

**The decision is deliberately unmade**, and here is the evidence it
needs:

| at the shoulder joint | mass carried |
|---|---|
| direct drive | bracket 20.9 g + servo 55.0 g = **75.9 g** |
| parallel linkage | bracket 48.4 g + servo at the base = **48.4 g** |

The linkage bracket is *heavier as a part* (48.4 g against 20.9 g — it
has no servo-sized cavity) and the joint still carries **36% less**,
because the servo is not there. That mass sits at the end of a lever, so
it costs more than its weight.

Against that: a parallel linkage is a **closed kinematic chain, which
URDF cannot express**. The simulation would become an approximation of
the robot, and every test, the FK checks, the tipping analysis and the
ML labels rest on the model being faithful. That is a real architectural
cost, not a modelling inconvenience.

Both get printed before anyone decides.

## The URDF's masses are now known to be wrong

| | |
|---|---|
| URDF says `upper_arm_link` | **55.0 g** (`provenance: estimated`) |
| printed beam, computed from geometry | **16.7 g** |

The estimate predates any geometry, and it cannot be right for both
mechanisms:

- **Direct drive** puts the elbow servo on this link: 16.7 + 55.0 =
  **71.7 g**, so the URDF *understates* it by 30%.
- **Linkage** leaves the servo at the base: about **20 g**, so the URDF
  *overstates* it by 175%.

One number cannot serve both. **The mass model depends on the mechanism
decision that has not been made**, which is a cleaner statement of the
problem than the roadmap's original note that 55 g "is about one MG996R
with nothing left over for structure".

Reconciling the URDF against generated geometry is the next honest step.
Until then the simulation is modelling a robot nobody has designed —
including the Phase 11 tipping margins, which were computed from these
estimates.

## What the tests catch

They are the cheap versions of mistakes that otherwise cost a print:

- **the part is a closed, positive-volume solid** — a cut that removed
  everything still exports happily, and the slicer produces nonsense
- **it fits a 220 mm bed** — a part needing a bigger printer is one most
  people following the course cannot make
- **no hole below the nozzle diameter** — the slicer drops it, the part
  looks right, and the screw has nowhere to go
- **M3 holes carry the printing clearance** — a nominal 3.0 mm hole does
  not take an M3 screw off an FDM printer, and modelling at nominal is
  the commonest reason a printed assembly needs a drill
- **the mechanisms actually differ** — a parameter that changes nothing
  is worse than no parameter, because the comparison the whole decision
  rests on would be between two identical parts

### One caught a real bug immediately

The first bracket was sized to the servo **body** (40.7 mm). The
mounting screws sit **49.5 mm** apart — wider than the body — so both
holes fell outside the material and cut nothing. The part still
exported, still looked correct, and would have been discovered at
assembly.

`test_m3_holes_carry_the_printing_clearance` found it by noticing the
only cylindrical faces in the part were the corner rounds. The bracket
is now sized to the flange span.

## The decision: direct drive

Chosen, on these numbers:

| | PLA | **lifted at the shoulder** |
|---|---|---|
| **direct drive** (chosen) | **168 g** | **140 g** |
| parallel linkage | 264 g | 85 g |

The linkage lifts 39% less at the joint with the least margin — a real
advantage, and not enough to pay for a **closed kinematic chain that
URDF cannot express**. Every test, FK check, tipping margin and ML label
in this repository rests on the model being faithful to the robot, and a
mechanism that forces the simulation into an approximation costs more
than the grams it saves.

It also needs 96 g more plastic and exceeds the 250 g filament budget.

The linkage remains generatable. The comparison that chose between them
should stay reproducible, and a parameter only ever exercised one way is
a parameter that has quietly stopped working.

### Choosing settled two design errors

Direct drive made the servo-to-link assignment unambiguous, and two
parts were immediately wrong:

- **`shoulder_bracket` had no job.** The turret carries the lift servo
  at the joint; the bracket duplicated it with a *vertical* servo
  pocket, which no axis on this arm wants — every joint but the pan is
  horizontal. It is now linkage-only, where it holds the pivot and rod
  anchor that replace that servo.
- **`wrist_bracket` held a servo it should not.** The wrist servo rides
  on `forearm_link` and the gripper's on `gripper_base_link`;
  `wrist_link` carries none. It would have printed, fitted a servo, and
  left that servo with no joint to move while the real one had nowhere
  to mount.

A third came from a test: **the turret had no mounting holes at all** —
nothing held the lift servo in, and nothing bolted the turret to the pan
servo's horn. A pocket is not a mounting.

## The masses are computed now, not guessed

For fifteen phases the link masses were guesses, because nothing had
been designed. They were wrong in **both directions**:

| link | was (estimated) | now (computed) |
|---|---|---|
| `base_link` | 250.0 g | **110.5 g** |
| `shoulder_link` | 60.0 g | **99.9 g** |
| `upper_arm_link` | 55.0 g | **73.5 g** |
| `forearm_link` | 45.0 g | **22.7 g** |
| `wrist_link` | 30.0 g | **6.3 g** |
| `gripper_base_link` | 35.0 g | **38.9 g** |

Base 2.3× too heavy, shoulder 40% too *light*. That is worse than being
uniformly wrong: a tipping margin computed from them could have been
optimistic or pessimistic and nobody could tell which.

A link's mass is its printed structure plus the servo that **rides** on
it plus that servo's horn adapter — and which servo rides where is read
from the joints, not a hand-written table, so re-parenting the arm moves
the mass with it. My first pass *did* hand-write that table and got two
links wrong, giving the wrist an adapter for a servo it does not carry.

`test_the_profile_masses_match_the_geometry` keeps them together: change
a part and the profile has to follow, or the simulation goes back to
modelling a robot nobody designed.

**`provenance: computed`** — a new value, kept distinct from `measured`,
because PLA density is nominal, servo masses are datasheet figures,
fasteners are not counted, and **nothing here has been weighed**.

The mobile manipulator now masses **1.136 kg** against the estimated
1.267 kg, and its worst tipping margin improves to **+43.2 mm** against
a required 21 mm.

The long-reach profile keeps `estimated` masses and says so in the file:
it is a kinematic variant with no `fabrication` block and no servo
envelopes, so there is no geometry to compute from. Inventing printer
data to make the numbers look consistent would be worse than the note.

## The CAD changed the robot
## The CAD changed the robot

The pan servo stands in the base with its shaft up. An MG996R is
**42.9 mm** tall. The base was **30 mm**.

Counting wall thickness and pocket clearance, it was short by **14.4 mm**
— the base could not contain the part it exists to hold. The simulation
never noticed, because a URDF box does not have to hold anything.

`base_link` is now 45 mm, and `shoulder_pan_joint`'s origin moved with
it. Three kinematics tests that carried hand-computed heights were
updated: the tool now sits at 0.300 m instead of 0.285 m at the home
pose. They stay hand-computed, because an independent cross-check is
worth more than one that re-derives the same number.

**This is the first time generating the CAD changed the robot rather
than describing it** — and it is the clearest argument for doing the CAD
at all.

## Printing

`make cad` writes `PRINTING.md` beside the parts, generated for the same
reason the parts are: a guide written by hand goes stale the first time a
part changes thickness, silently, because nothing compares the two.

It carries the slicer settings from the profile, the per-part
orientation, the **quantities** (2 fingers, 4 bushings — a part file says
nothing about how many are needed), and the fastener count.

The fastener count is derived from the holes, and is deliberately **not**
a shopping list. A hole through a hollow part appears twice, two coaxial
holes in two parts are one screw, hole depth is not screw length, and
nobody has decided between nuts, heat-set inserts and self-tapping
screws. A confident schedule from half the information would be worse
than saying "buy an M3 assortment".

## The printed parts must build the robot the URDF describes

A beam of the right length can still assemble into a different machine.

The shoulder-lift axis is **horizontal** — the URDF says
`axis: [0, 1, 0]` — so that servo's shaft points along Y, which means it
**lies on its side**. The first turret stood it upright like the pan
servo below it, which would have put the lift axis about **44 mm** up
instead of the **20 mm** the URDF places it at.

Nothing in the bracket's own geometry reveals that. It looks like a
perfectly good bracket. It just assembles into a different robot than
the one every trajectory, FK test and tipping margin was computed for —
and it would have been found by holding the arm up and noticing it was
the wrong shape.

`test_the_lift_axis_sits_where_the_urdf_puts_it` now checks the shaft
clearance against the joint origin, and companion tests check the beams
against their joint spacing and the turret against its own bore.

Getting there took three more geometry fixes, each found by the
printability check rather than by inspection: the servo cavity opens at
the top (a closed one left a 0.7 mm ceiling spanning 20.3 mm), and the
bearing-seat bore runs the full height (stopping short left a 53 mm
roof with one servo, and a 226 mm strip between cavities with two).

## The assembled arm, in one file

`make cad` writes `assembly.step` — every part placed at the joint
origin the URDF gives it, at the home pose.

Every other check here tests **one number at a time**: a bore against a
bushing, a beam against a joint spacing, a shaft against an origin. All
of them passed while the turret held its servo the wrong way up, because
no single measurement was wrong — the part was simply the wrong shape,
which a person sees in a second and a test does not see at all.

Opening that file is the cheapest review the design gets before filament
is spent.

It also enables the one check that needs the whole arm rather than one
joint: **pairwise intersection**. Parts can each be correct and still
collide once placed — the turret reaches up past the lift axis and the
upper arm starts at it, so the two are a few millimetres from sharing
space. Currently none overlap.

The assembly is **not** a kinematic model. The URDF is, and it stays the
source of truth; this reads its origins and puts plastic at them. If the
two disagree, the URDF is right, because it is what the controllers, the
planner and every test actually use.

The arm stands **285 mm** at the home pose.

## What is still not designed

**Nothing has been printed.** These parts pass geometric, fit,
printability and stack-up checks — which is still a different claim from
parts that fit together.

Two things no geometric check can verify, both cheap to test first: the
**press fit** of a bushing and the **clearance** of the thrust washer.
Print one of each before committing to a full set.

The **horn adapter** rests on `horn_provenance: estimated` dimensions
and is where the first print will find trouble. Measure the horn that
arrives.

An optional real **thrust bearing** for the pan joint is still not in
the BOM. The washer works; a bearing works better.

## The toolchain costs 772 MB

build123d, essentially all of it OpenCascade via `cadquery-ocp`. There is
no lighter path to real B-rep solids with STEP export, and exporting only
STL is how an "open" design becomes un-editable.

It lives in its own `cad` Dockerfile stage, in a **venv**: build123d
wants a newer `typing_extensions` than Debian ships, and pip cannot
uninstall a dpkg-managed package, so a plain install fails part-way and
leaves the system Python in a state neither tool owns.
`--system-site-packages` keeps yaml, numpy and ROS visible, so one
interpreter has both.

It is in the CI stage deliberately: CAD tests that only run on one laptop
are CAD tests that rot.

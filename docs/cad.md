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

## The number the decision turns on

Every structural part now generates for both mechanisms, so the
comparison can be made on the quantity that matters rather than the one
that is easy to measure:

| mechanism | PLA total | **lifted by the shoulder servo** |
|---|---|---|
| direct drive | 161 g | **154 g** |
| parallel linkage | 257 g | **99 g** |

The linkage needs **96 g more plastic** and lifts **36% less** at the
joint with the longest lever and the least margin. Total printed mass is
the number that favours direct drive, and it is nearly irrelevant — a
gram at the base costs nothing and a gram at the wrist costs a moment
arm.

Most of that extra plastic is one part: the linkage turret is 93.5 g
against 24.7 g, because it carries **two** servos instead of one. That
is not a side effect, it is the mechanism — moving the elbow servo down
there is the whole reason to accept a closed chain.

`test_the_shoulder_servo_can_lift_the_arm_it_has_to_lift` closes the
loop between the CAD and the physics: it takes the parts distal to the
shoulder, adds the servos riding on them and the 50 g payload, puts the
lot at full reach, and compares the moment against a derated MG996R.
Both mechanisms pass; the linkage has far more margin.

### The linkage blows the filament budget

257 g against the 250 g the BOM assumed. Small, and worth saying out
loud rather than rounding away: the BOM now carries **measured** figures
for both variants instead of a projection made before any part existed.

## The URDF masses, revisited

With every part generated, the gap is no longer one link:

| link | URDF says | printed structure |
|---|---|---|
| `upper_arm_link` | 55.0 g | 16.7 g |
| `forearm_link` | 45.0 g | 12.6 g |
| `base_link` | 250.0 g | 33.7 g |
| `gripper_base_link` | 35.0 g | 28.5 g |

Every one is `provenance: estimated` and every one is wrong, some by 7×.
They still cannot simply be replaced, because a link's *total* mass
includes whatever servo rides on it — and that depends on the mechanism
nobody has chosen yet.

The Phase 11 tipping margins were computed from the estimated column.

## The assembly

The parts that join the other parts — and most of them fail in ways a
bracket cannot: a rod of the wrong length still moves, a bushing that is
a slip fit still assembles.

### The spline is not printed, and here is the arithmetic

A 25T spline on a 5.9 mm shaft has a **0.74 mm tooth pitch** — **1.9
extrusions** across a tooth at a 0.4 mm nozzle. The tooth form is
unresolvable at that scale, and a printed spline driven by a servo
delivering 0.9 N·m is not a part, it is a consumable.

Every hobby servo ships a **metal horn**. `horn_adapter` bolts to it,
takes its screws, and presents a flat interface to the driven part. The
metal carries the torque; the plastic only locates. A recess receives
the horn so the adapter seats on the servo boss rather than perching on
the horn's rim — the difference between a joint with a defined axis and
one that rocks.

That reasoning lives in `test_the_spline_is_too_fine_to_print` rather
than a comment, so that a coarser spline or a finer nozzle makes the
number change and the decision get revisited.

**The horn dimensions are the most likely numbers in this repository to
be wrong.** Horns vary between manufacturers even for a servo sold as an
MG996R, so they carry `horn_provenance: estimated` and a test asserts
they still claim no better. Measure the horn that arrives before
printing the adapter.

### The push rod's length is not free

A parallel linkage works because the rod and the link it parallels form
a **parallelogram** — equal and parallel sides — so the forearm holds
its angle as the shoulder moves. The rod length therefore comes from the
same `upper_arm_link` entry the URDF reads.

Get it wrong and the mechanism still moves. It just stops being a
parallelogram: the forearm angle drifts with shoulder angle, and the
arm's kinematics quietly stop matching any model of it. Two tests pin
it, one on the outline and one on the **bore centres**, because the
bores are what define the linkage.

`push_rod` raises for direct drive rather than returning something,
which would put a rod in a box of parts with nothing to connect it to.

### Bushings and a washer, not bearings

`pivot_bushing` is a **sliding fit inside and an interference fit
outside**. Backwards, and it spins in its bore while gripping the screw,
wearing the bracket instead of the sacrificial part — the exact outcome
it exists to prevent. Printed plastic on a steel screw wears *oval*
rather than staying round, so the slop appears in one direction and
reads as a wobbly arm rather than a worn bearing.

`thrust_washer` is **not a bearing**, and a test asserts the docstring
says so. A real thrust bearing is a bought part; this is what makes the
joint work without one, by putting the wear somewhere replaceable.
"Printed bearing" is how a design acquires a reputation for slop.

## Every part, both mechanisms

| | PLA total | **lifted by the shoulder servo** |
|---|---|---|
| direct drive | 165 g | **154 g** |
| parallel linkage | 265 g | **99 g** |

Twelve parts for direct drive, thirteen for the linkage — the rod is the
linkage in the same way a servo pocket is direct drive, so the registry
is mechanism-aware rather than letting a part raise for a reason that is
not an error.

## What is still not designed

**Fasteners are bought, and not yet counted.** The design has the holes;
nobody has derived a screw schedule from them, and guessing one would be
a BOM line with no evidence behind it.

A **real thrust bearing** for the pan joint is optional and not in the
BOM. The washer works; a bearing works better.

**Nothing has been printed.** These are solids that pass geometric
checks, which is a different claim from parts that fit together. The
first print will find things no test here can — and the horn adapter,
resting on estimated dimensions, is where it will start.

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

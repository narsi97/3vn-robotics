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

## What is still not designed

The parts exist. **The assembly does not.**

Missing: the push rods for the linkage variant, the bearing itself, the
servo horn adapters, and the fasteners. Those are bought parts and joint
details, and the horn adapter in particular is the fiddliest piece of a
printed arm — a 25T spline is not something to model casually.

**Nothing has been printed.** These are solids that pass geometric
checks, which is a different claim from parts that fit together. The
first print will find things no test here can.

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

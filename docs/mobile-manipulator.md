# Mobile manipulator

```bash
make mm-sim                # headless
make mm-sim GUI=1          # Gazebo GUI at http://localhost:8106/vnc.html
make mm-verify             # drive and manipulate at once, against a running sim
make urdf ROBOT=mm         # dump the expanded description
```

The arm, on the base. **Composed, not merged** — neither component
description was modified, and both still build and run alone:

```bash
make sim         # the arm
make base-sim    # the base
make mm-sim      # both
```

A test asserts each component still expands standalone. If mounting one
on the other had required editing either, Phase 11 would have produced a
third robot to maintain instead of assembling two that already worked.

This is what Phase 1 and Phase 10 were buying when each spent a fixed
joint on a mount frame nobody used yet.

## Two configs in one document

Every macro under `urdf/arm/` and `urdf/base/` reads a property called
`cfg`. That is fine while a document describes one robot and breaks the
moment it describes two: whichever config was assigned last wins, and
the arm gets built out of base numbers — silently, because every lookup
that happens to exist in both succeeds.

Rebinding `cfg` between the two macro calls *does* work, since xacro
expands in document order. It was rejected anyway: reordering the calls
or adding a third subsystem would quietly build the wrong robot, which is
the exact failure mode Phase 10 was spent removing from the test harness.

Instead each subsystem is wrapped in a macro that sets `cfg` **inside its
own body**. Xacro scopes properties to the macro invocation, so the arm's
macros see the arm's config and the base's see the base's, with no
ordering dependency and no change to either component.

## One controller manager

Both component descriptions used to emit the `gz_ros2_control` system
plugin themselves. Invisible while each robot had one subsystem, and
wrong as soon as they were composed: **two** controller managers in one
simulator process, both named `controller_manager`, both loading the
same controllers file.

A `<ros2_control>` block describes a set of interfaces and may appear
many times. The system plugin *hosts* the controller manager and must
appear once. Those are different things, so they now live in different
files — `ros2_control/gz_control_plugin.xacro`.

The composed robot runs four controllers on one manager:

```
arm_controller           joint_trajectory_controller   active
gripper_controller       position_controllers          active
base_controller          diff_drive_controller         active
joint_state_broadcaster  joint_state_broadcaster       active
```

## The interface does not change

Only joint *names* gain the `arm_` prefix. Controller names do not, so
every topic and action is the same on the composed robot as on either
component alone:

| | |
|---|---|
| `/joint_states` | one message, all four arm joints and all four wheels |
| `/base_controller/cmd_vel` | unchanged |
| `/arm_controller/follow_joint_trajectory` | unchanged |

`make mm-verify` drives and manipulates **concurrently**, which is the
part worth checking — run sequentially, two controllers that quietly
share a resource still look fine:

```
ok    one /joint_states carries all 4 arm joints and all 4 wheels
ok    the arm goal was accepted while the base was driving
ok    the base travelled 0.345 m while the arm was moving
ok    arm_shoulder_pan_joint moved 0.396 rad while the base was driving
```

## Controller parameters are merged at launch, not committed

There is no `mm_controllers.yaml`. The launch file reads the arm's
`controllers.yaml` and the base's `base_controllers.yaml` and applies the
prefix to the arm's joint names.

A committed merged file was the obvious move and the wrong one: the same
joint list would exist in two places, and adding a joint to the arm would
leave the composed robot silently driving the old set. Same reasoning
that keeps the URDF expanding at launch rather than being generated and
committed.

The merge **refuses** rather than resolves a disagreement. Both configs
declare `controller_manager.update_rate`; letting one silently win would
mean the robot ran at a rate neither file states, and the arm's tuning
would stop matching its own loop. Tested in
`threevn_sim/test/test_controller_merge.py`, which needs no ROS graph.

## Can it tip itself over?

A mobile manipulator can throw itself down. The arm is a large fraction
of the total mass and it **moves**, so the centre of mass is a function
of the joint angles — which makes "does it tip" a question about the
whole reachable workspace, not about one pose.

`test_the_robot_cannot_tip_itself_over` sweeps the arm's joint limits
carrying the declared payload at the grasp frame, and checks the centre
of mass against all four wheel contacts. No simulator: geometry and
arithmetic over the same URDF that runs.

| | mass | reach | worst margin | required |
|---|---|---|---|---|
| `threevn_mm_v1` | 1.267 kg | +0.244 m | **+38.8 mm** (rear) | 21.0 mm |
| `threevn_mm_v1_long_reach` | 1.313 kg | +0.338 m | **+30.0 mm** (rear) | 21.0 mm |

Two things the measurement corrected.

**The binding edge is the rear, not the front.** The support rectangle is
narrower front-to-back (wheels at x = ±0.070) than side-to-side
(±0.085), so reaching forward looks like the danger — and mounting the
arm behind centre was supposed to buy margin against it. It does, and it
spends that margin at the rear, because the resting centre of mass moves
back with the arm. The sweep checks all four edges for exactly this
reason.

**Reach pulls against stability.** Both arms reach well past the chassis
front edge at 0.100 m, which is what makes the arm useful rather than
something that can only work on its own roof. Every millimetre of reach
is also a millimetre of leverage, and the long-reach arm has 8.8 mm less
margin for 94 mm more reach.

A margin is required rather than bare containment. A centre of mass
exactly over an axle is already unrecoverable: braking, a floor seam or
the arm's own momentum finishes it.

Every mass in this model is `provenance: estimated`. These margins are
arithmetic on guesses and must be re-run once anything is weighed.

## A second composition, for the same reason as before

`threevn_mm_v1_long_reach` composes the *same base* with the long-reach
arm. It shares every line of xacro with `threevn_mm_v1` and differs only
in which arm it names — the same trick `threevn_arm_v1_long_reach` plays
for the arm. If composing a different arm ever needed a xacro edit, the
composition would not really be parameterised, and the tests say so.

It is also the demanding case: a longer arm puts the same payload further
out, and the support rectangle did not get any bigger.

## Naming

The base owns the unprefixed names, because **in a composed robot the
base is the robot**: its `base_link` is what REP-105 means and what nav2
navigates, and its `base_footprint` is the kinematic root. The arm's own
`base_link` becomes `arm_base_link`.

That is not cosmetic. Both descriptions declare an `imu_link`, and two
links with one name is not a parse error — `urdf_parser_py` builds a dict
keyed by name, so the second silently wins and the tree still looks
fine. A test asserts both survive.

## Still to do

`threevn_navigation` and Nav2 are **not** in this phase. The composed
robot drives and manipulates; it does not yet plan a path or build a map.
The frames navigation needs (`base_link`, `odom`, the wheel odometry and
its deliberately pessimistic covariances) are in place, and heading
remains the weakest state the robot publishes — see
[mobile-base.md](mobile-base.md).

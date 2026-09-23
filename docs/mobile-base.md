# Mobile base

```bash
make base-sim              # headless
make base-sim GUI=1        # Gazebo GUI at http://localhost:8106/vnc.html
make base-drive            # measure odometry against ground truth
make urdf ROBOT=base       # dump the expanded description
```

Four wheels, **skid steer** — which is what cheap chassis kits are: four
TT gearmotors, or two driving through a belt. `diff_drive_controller`
handles two wheels per side natively, so there is no custom controller.

## It is a separate robot, deliberately

Phase 10 builds the base **standalone**. Phase 11 composes it with the
arm; it does not merge them.

- `threevn_base.urdf.xacro` references nothing from the arm — a test
  asserts it.
- The base exposes `arm_mount_link`; the arm exposes `mount_plate_link`.
  Neither knows about the other.
- Separate controller config, separate `ros2_control` block, and in
  Phase 5 terms a **separate ESP32 on its own port**. One
  microcontroller driving both would couple their failure modes, and a
  base firmware fault should not take the arm down.

That separation is what lets each be developed, tested and deployed on
its own.

## Wheels are velocity-commanded, not position

You do not tell a wheel where to be. That is why the base uses
`diff_drive_controller` rather than the arm's trajectory controller, and
why its joints expose a `velocity` command interface. A position
interface here is a category error that only shows up at runtime.

The joints are **continuous**, not revolute. Declaring a travel limit on
a wheel makes the controller fight it after a few metres, presenting as
a base that gradually stops for no visible reason.

## Odometry on skid steer is approximate, and that is not fixable

Turning requires the wheels to **scrub sideways**, which the differential
model does not represent. Heading drifts fastest, in proportion to how
much turning has happened rather than distance travelled.

Measure it yourself:

```bash
make base-sim          # in one shell
make base-drive        # in another
```

`base-drive` compares wheel odometry against **Gazebo ground truth**, and
that distinction is the whole point. Comparing odometry to the *command*
proves only that the controller tracks its setpoint — both sides of that
comparison come from the same wheel encoders, so neither can see slip. A
base that is physically bolted to the floor still reports a perfect turn.

What the measurement shows, commanding 0.9 m forward and a 114.6° turn:

| leg | odometry | ground truth | error |
|---|---|---|---|
| forward 0.9 m | 0.882 m | 0.882 m | **0.0 %** |
| rotate 114.6° | 112.3° | 89.0° | **+26 %** |

Straight-line odometry is essentially exact. Heading is not, and the gap
is the scrub.

### The correction saturates

`wheel_separation_multiplier` is the empirical fix: it widens the track
the controller believes it has. Measured:

| multiplier | actually turned | odometry error |
|---|---|---|
| 1.00 (ideal differential) | 70.4° | +62.7 % |
| 1.63 | 84.5° | +34.3 % |
| **2.40** (shipped) | 89.0° | +26.2 % |

It flattens. The multiplier scales the command *and* the odometry
together, so it can shrink the error but never close it — a single linear
track-width constant cannot represent real scrub. Roughly a quarter of
the commanded heading is simply lost.

That is the honest limit of wheel odometry on this chassis, and the
reason the base carries an `imu_link`: heading has to come from a sensor
that measures rotation directly. The covariances in
`base_controllers.yaml` say the same thing in a form a localisation
filter can use — leaving them optimistic is how navigation fails later
for reasons that look unrelated.

On a real floor these numbers will differ; carpet and lino do not scrub
alike. Re-measure in Phase 11.

### Measure against a fresh simulator

A base left sitting after an earlier run has reported wheel speeds near
zero against a 2.58 rad/s command, and a pose frozen to sixteen decimal
places. That looks exactly like a physics finding and is not one — it is
a degraded Gazebo state. Restart `base-sim` before trusting a number.
This cost real time during Phase 10: an early hand measurement produced a
"12 % shortfall" that was an artifact of a reused simulator, not
geometry.

## Three bugs the tests caught that the simulation hid

**The base had no `base_link`.** Its main body was called `chassis_link`,
so the robot's root chain was `base_footprint → chassis_link` and
`base_link` did not exist at all. REP-105 defines `base_link` as the
required frame rigidly attached to the robot base, and nav2 defaults
`robot_base_frame` to it — so this would have surfaced in Phase 11 as
navigation failing for reasons that looked nothing like a naming problem.
Nothing caught it until the shared description tests learned to check
required frames per robot family. In a composed mobile manipulator this
is `base_link`; the arm's own takes the `arm_` prefix.

**The chassis was buried 20 mm into the floor.** The ride-height
calculation wrongly subtracted half the chassis height, putting the axles
at 0.013 m against a 0.033 m wheel radius. Gazebo looked completely fine
because the spawn used `z=0.05`, which lifted the robot clear. Only
`test_the_chassis_sits_on_its_wheels` caught it — and once fixed, the
spawn height went to 0 and the measured travel changed, because the
wheels were finally touching the ground.

**The wheel velocity limit exceeded the motor.** 20.0 rad/s configured
against a ceiling of 19.95 (21.0 no-load × 0.95 derate). Same class as
the Phase 5 gripper force, and caught the same way: by asserting joint
limits against the motor that has to deliver them.

## A stale parameter that silently did nothing

`base_controllers.yaml` originally set `use_stamped_vel: false`.
**That parameter does not exist** on Jazzy's `diff_drive_controller` — it
was accepted and ignored, and `/base_controller/cmd_vel` is
`TwistStamped` regardless.

The symptom was a base that would not move, with no error in any log: a
plain `Twist` publisher simply never matched the subscriber.

```bash
ros2 topic info /base_controller/cmd_vel     # the fastest way to check
```

## Friction is tuned, not default

Skid steer turns *by* sliding sideways, so `mu2` (across the rolling
direction) is deliberately much lower than `mu1`. Setting them equal is
the usual mistake and produces a robot that drives forward but refuses to
rotate.

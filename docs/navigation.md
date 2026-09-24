# Navigation

```bash
make mm-sim WORLD=bench_with_target
make fused-odom      # heading from the gyro
make heading         # compare all four sources against ground truth
```

## Nav2 is not installed, and that is the finding

The roadmap assigned Nav2 to Phase 11. Installing it would have been the
easy part. It was not done, because **this robot has exactly one sensor:
a camera.**

| Nav2 needs | this robot has |
|---|---|
| a laser scan for AMCL | nothing |
| obstacle sensing for costmaps | an RGB camera, no depth |
| trustworthy heading | wheel odometry, measured 30% short |

Nav2 would install, launch, and produce a robot that localises from
wheel odometry and drives confidently into walls. That is worse than not
having it, because it looks like navigation.

A range sensor is the honest prerequisite, and it is a **budget
decision, not a software one**: an RPLIDAR A1 is around $100 against a
whole-robot BOM of $45–55. Tripling the cost of the machine to add
navigation is a real choice, and the course should present it as one.

So this phase does the part that is both cheap and strictly necessary
first.

## Heading, measured

Phase 10 established that wheel odometry over-reports heading by 63% on
this chassis, and that `wheel_separation_multiplier` — the standard
correction — **saturates around 26% error**, because skid-steer scrub is
not a fixed track-width error. No value of that constant fixes it.

An IMU was added: an MPU6050 or equivalent, **about $2**, the cheapest
part on the robot. Driving four turns and comparing against Gazebo
ground truth:

| leg | wheel odom | gyro | fused | truth |
|---|---|---|---|---|
| +0.8 rad/s × 2 s | +42.2° | +61.0° | +61.0° | **+61.1°** |
| −0.8 rad/s × 2 s | −0.1° | +1.9° | +1.9° | **+2.2°** |
| +0.6 rad/s × 3 s | +47.3° | +65.1° | +65.1° | **+65.6°** |
| −0.6 rad/s × 3 s | −4.4° | −5.7° | −5.7° | **−5.0°** |

| source | mean per-leg error | worst |
|---|---|---|
| wheel odometry | **17.6°** | 18.9° |
| gyro integral | **0.2°** | 0.3° |
| fused odometry | **0.2°** | 0.2° |

A $2 part does roughly **88× better** than the calibration constant that
provably could not.

### The first metric was misleading

The summary originally reported *net* error after a sequence of turns
netting zero rotation. By that measure one run gave wheel odometry
**+0.4°** against the gyro's −0.6° — the wheels looking better.

Wheel heading error is **proportional to rotation**, so equal and
opposite turns cancel it. Every individual leg was 30% short while the
total looked perfect. Per-leg absolute error is the honest measure and
the script now leads with it.

This is the second time in this project a summary statistic quietly
stopped measuring the thing: Phase 13's label-distance leakage metric
did the same when the dataset grew.

## Heading from the gyro, distance from the wheels

`fused_odom` publishes `/odometry/filtered`. Each source is used for
what it is measurably good at — straight-line wheel odometry matched
ground truth to **0.0%** in Phase 10, so there is nothing to improve
there.

Position uses the wheel *distance* carried along the *gyro* heading,
not the wheel odometry's own x and y. A differential model that thinks
it turned 45° when it turned 68° travels the right distance in the wrong
direction, and the error compounds with every leg even though the
odometer was right.

The covariances are reversed from the wheel-only case: heading is now
the well-known quantity and position the one that accumulates.

### It does not publish TF by default

`diff_drive_controller` already owns `odom → base_footprint`. Two
publishers of one transform is a fight nobody wins — consumers see
whichever arrived last, the robot appears to jitter between two answers,
and nothing in any log says why. Taking TF over means setting
`enable_odom_tf: false` on the controller in the same change, so
`publish_tf` is a parameter and starts `false`.

### What the bias estimate is for, and what it is not

A gyroscope at rest reads a small constant offset. Integrating it turns
degrees-per-minute of drift into degrees-per-second, so the fuser
measures it while the robot is known still.

*Known still from the wheels*, not from the gyro. Using the gyro to
decide whether to trust the gyro is a loop that calibrates away a real
slow rotation.

**It still drifts.** A gyro integral has no absolute reference, so its
error is a random walk that grows without bound. This robot has no
magnetometer, no scan matcher and no GPS — nothing here corrects it. The
claim is much slower drift, not bounded drift.

`robot_localization` is the standard tool and would be right with more
sensors to fuse. With exactly one rotation source there is nothing to
weigh against anything, and an EKF over a single input is a covariance
matrix wrapped around a subtraction.

## Two Gazebo traps

**The IMU needs its own system plugin.** `gz-sim-sensors-system` drives
*rendering* sensors — cameras, depth, lidar — and does not touch an IMU.
Without `gz-sim-imu-system` in the world, the sensor parses, the bridge
is created, and `gz topic -i` reports *"No publishers on topic [/imu]"*.
Every layer looks configured and nothing produces a reading.

**The IMU is simulated with noise, deliberately.** A noiseless gyro
integrates to a perfect heading and makes fusing it a foregone
conclusion. The values come from the MPU6050 datasheet, so the drift
seen here is the drift the $2 part actually has.

## What is still missing

- **A range sensor.** Without one there is no mapping, no obstacle
  avoidance and no absolute heading reference. This is the blocker for
  Nav2 and it costs money, not effort.
- **Nav2 itself**, which follows the sensor.
- **TF ownership**, one config change away and deliberately not taken.
- **Everything physical.** The IMU is a line in a YAML file; no robot
  has been built.

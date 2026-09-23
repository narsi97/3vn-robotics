# Perception

```bash
make mm-sim WORLD=bench_with_target   # the robot, and a green cube
make perception                       # find it
make perception-verify                # how wrong is the answer
```

The camera frames were created in Phase 1 and left empty on purpose: no
`<sensor>` was emitted, so every headless test avoided paying for
CPU-rendered images it never read. Phase 12 turns the camera on, and the
frame convention those links encode finally does something.

## The detection is arithmetic, not a pipeline

Everything that decides anything lives in
[`detector.py`](../src/threevn_perception/threevn_perception/detector.py):
pure functions over a `(h, w, 3)` uint8 array. No rclpy, no topics, no
simulator. The 30 tests draw their own images and run in under a second.

The ROS node is deliberately thin — subscribe, transform, publish.

This split is not tidiness. Perception is exactly where a swapped axis or
a wrong sign produces output that is plausible in every respect and wrong
by ninety degrees, and a detector you can only exercise by starting
Gazebo and waiting for a frame is a detector nobody tests.

## Why the target is green

The robot's livery is **orange** (`0.90 0.49 0.13`), which in hue terms
sits close enough to red that a red target makes the detector lock onto
the arm — intermittently, as the arm swings through frame, which is the
worst way for a test to fail.

A parametrized test feeds the detector each of the robot's own colours
and the ground, and requires that none of them is detected.

The mask tests **dominance**, not brightness: a channel must exceed the
other two by a margin. A plain threshold on the green channel calls every
specular highlight a target, and a rendered scene is full of them.

## Connectivity matters

Taking the centroid of every masked pixel is the obvious shortcut. With
two objects of the same colour it puts the answer midway between them, on
empty ground, and the result looks like an ordinary detection in every
respect except being of nothing at all. `largest_blob` does an iterative
flood fill and returns the biggest region; a test places two squares and
requires the answer to be the larger one.

## Which frame the image is in

Gazebo points a camera along **+X** of the link carrying it. ROS optical
convention (REP-103) is **+Z forward, +X right, +Y down**. Both are
right, and they are not the same.

That is why `camera_link` and `camera_optical_frame` are separate links
with a fixed joint carrying exactly that rotation — decided in Phase 1,
when it cost one joint. The sensor hangs off `camera_link` so Gazebo's
convention applies to the rendering, and `gz_frame_id` names the
**optical** frame, because that is what the published `Image` and
`CameraInfo` describe.

Naming `camera_link` there instead produces a pipeline that runs,
publishes at the right rate, reports plausible numbers, and sends the arm
to the wrong place.

One consequence that looks like a bug: image `v` increases downward and
optical `+y` is also down, so they share a sign and there is **no flip**.
The code that looks like it forgot one is correct.

## Intrinsics come from the camera, not the config

`fx = (width / 2) / tan(hfov / 2)`. With 640 px and 62°, that is 532.57 —
and the simulator publishes exactly 532.57.

The node still reads `CameraInfo` rather than recomputing from the robot
profile. The profile says what the camera *should* be; `CameraInfo` says
what it *is*. A test asserts the two agree, which is how a resolution
change that never reached the simulator gets noticed.

## How wrong is it?

The cube's true pose is in the world file and the robot has not moved, so
the truth is known exactly:

| axis | detected | true | error |
|---|---|---|---|
| x | +0.370 m | +0.400 m | **−30.2 mm** |
| y | +0.000 m | +0.000 m | +0.3 mm |
| z | −0.012 m | −0.018 m | +5.7 mm |

**Bearing is excellent, range is biased, and they fail for different
reasons.**

The centroid is robust. Sweeping the colour threshold across its usable
band (dominance 20→75) moved the horizontal centroid by **0.00 px**: a
soft edge fades symmetrically, so tightening the threshold removes the
same ring of pixels all the way round.

The **width** is measured at the edge, which is where the ambiguity
lives. Rendered boundary pixels blend into the background, and
segmentation includes roughly 3 px of bleed per side — 74 px measured
against 68 px true. That 6 px is the entire 8% shortfall. Nothing in the
pinhole arithmetic is wrong; its input is.

Because the bleed is an edge effect it stays a few pixels wide as the
target shrinks with distance, so **the proportional range error grows**.
Both properties are pinned by tests.

### The first explanation was wrong

The initial guess was that a cube seen off-axis shows more than one face
and so looks wider. Measuring killed it: the cube is dead ahead of a
camera on the centreline, its silhouette is one face, and the bias is in
the segmented edge. The threshold sweep is what settled it — and it is
in the repo, so the claim can be rechecked rather than believed.

## Known limits, stated rather than hidden

- **Monocular range needs the object's size known in advance.** It works
  for a cube of declared dimensions and not for an arbitrary object. A
  depth camera or a second viewpoint removes the assumption; neither
  exists in this phase.
- **No orientation.** A colour blob carries no rotation. The published
  pose has an identity quaternion, which is *not* a measurement — three
  numbers should not pretend to be six.
- **No sensor noise.** The simulated camera is noiseless, deliberately:
  Phase 12 is about geometry and plumbing, and adding noise now would
  make every test flaky for reasons unrelated to what it checks.
- **Colour thresholding is not object recognition.** It finds a
  saturated hue, and it would find a green mug just as happily. Learning
  what a thing *is* belongs to Phase 14.

## Cost

640×480 RGB at 15 Hz is about 14 MB/s through the bridge, rendered on
**llvmpipe** — CPU, no GPU. The update rate lives in the robot profile
next to a note saying not to raise it without measuring. Nothing
subscribes to the image by default.

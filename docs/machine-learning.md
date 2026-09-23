# Machine learning

```bash
make mm-sim WORLD=bench_with_target
make dataset EPISODES=40     # 480 frames, ~25 min
make train                   # ~60 s to featurise, then instant
```

## The task already had a solution

Phase 12 deprojects a colour blob in closed form. This phase asks
whether fitting a model does better, and makes the answer falsifiable:
the baseline is evaluated **in the same run, on the same test set, by
the same function**. "The model beat the geometry" is not a claim until
both numbers come out of one place.

| | test median |
|---|---|
| closed form | 106.8 mm |
| fitted, geometry features | 46.5 mm |
| fitted, **+ shape features** | **10.5 mm** |

Ten times better than the closed form — for a reason that was measured
before the model was written.

## Why not raw pixels, and why not a neural network

A 640×480 RGB frame is 921,600 numbers and this dataset has 480 of them.
Fitting to that needs either a convolutional architecture and far more
data, or a smaller representation. The smaller one is free: the Phase 12
detector already reduces a frame to a blob.

And the mapping from that blob is **known to be linear** in the right
features. The deprojection is

```
z = fx·W / w        x = (u−cx)·W / w        y = (v−cy)·W / w
```

so every output is linear in `(u−cx)/w`, `(v−cy)/w`, `1/w`. A linear
model over those can reproduce the geometry exactly *and* fit `W`, which
the closed form takes as given. A network could at best rediscover the
same mapping while adding capacity to memorise 480 samples.

There is no gradient descent, no epochs and no learning rate: for a
linear model the normal equations have a closed-form solution, and
anything iterative would be ceremony that can also be tuned into looking
good.

*If the features were raw pixels the answer would be different, and so
would the amount of data needed.*

## What the closed form gets wrong

`W` is taken as the cube's 50 mm edge. Measuring the width the images
actually imply, `w·z/fx`:

| | implied target width |
|---|---|
| median | 64.9 mm |
| 10th–90th percentile | 55 – 78 mm |
| declared | 50 mm |

That brackets exactly **[50 mm, 70.7 mm]** — one cube face up to the
diagonal of two. Seen at an angle, a cube shows two faces and its
silhouette widens. `W` is not a constant; it varies with viewing angle,
and **nothing in the geometric features says which way the cube is
facing**.

So the fix was a hypothesis with a shape: give the model something that
carries the viewing angle. A blob's aspect ratio does — a square-on cube
is as tall as it is wide, a rotated one is not. The interaction terms
(`(h/w)/w`, `sqrt(area)/w²`) exist because the correction is
multiplicative: `z = fx·W(angle)/w` needs `1/w` *scaled* by the angle
term, not added to it.

The ablation tests that hypothesis rather than assuming it. Adding shape
took the test median from 46.5 mm to 10.5 mm.

### This reverses a Phase 12 conclusion, and both were right

Phase 12 proposed silhouette widening to explain its 30 mm shortfall,
measured, and **rejected it**: the cube was dead ahead of a camera on
the centreline, so its silhouette was one face, and the bias was edge
bleed in the segmentation.

That was correct for that measurement. Once the target moves off-axis,
the effect Phase 12 ruled out becomes the dominant one. Both mechanisms
are real; which dominates is a property of the data, not of the optics.

It is also why the baseline reads 106.8 mm here against Phase 12's
30 mm. Same code, same camera — a workspace instead of one favourable
spot.

## Read the coefficients with care

The `1/w → z` weight alone implies a 54.8 mm target. That is a tempting
sentence and a weak one: the features all share `1/w`, so they are
correlated (condition number ≈ 300) and the split of the mapping between
them is not unique. **A single coefficient is not a physical quantity
here**, even though the predictions are good.

The identifiable statement is the one measured directly from the data —
the implied width of 64.9 mm spanning 55 to 78 mm. When a fitted
parameter and a direct measurement disagree about what they mean, trust
the measurement.

## What these numbers are worth

Less than they look.

- **The test set is optimistic.** Phase 13 measured that this scene's
  episodes look alike: a flat grey floor and one small cube, so pictures
  from different episodes are genuinely similar. The split is honest —
  by episode, zero identical images in test — but the held-out data is
  not very held out.
- **The median hides the tail.** Test median 10.5 mm against a test p95
  of **74.8 mm**, and train p95 of 40.1. A robot reaching for something
  experiences the tail too.
- **Test scores better than train** (10.5 vs 14.9 mm median). With 71
  test frames from a handful of episodes that is variance, not skill,
  and it is a reminder that a single split of a small dataset is a noisy
  instrument.
- **Everything is simulated.** Noiseless camera, one target, one colour,
  no occlusion, no clutter, no lighting variation.
- **Eight frames of 480 were dropped** because the detector saw nothing.
  They are excluded rather than imputed — a frame the detector cannot
  see is not a training example, and inventing a feature means fitting
  the invention.

## The artifact

`model.json` carries the weights, the **feature names in order**, the
metrics for every split, the ablation, and the identity of the dataset —
by its manifest, not its path. A path says where a file was on one
machine; the manifest says what was in it.

Feature order is part of the contract: feeding a model its columns in a
different order than it was fitted on produces predictions, not an
error. That is what Phase 15 has to version and deploy.

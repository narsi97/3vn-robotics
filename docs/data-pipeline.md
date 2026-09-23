# Data pipeline

```bash
make mm-sim WORLD=bench_with_target   # in one shell
make dataset EPISODES=8               # record
make dataset-describe                 # what did we get, and is it honest
```

Datasets land in `datasets/`, which is gitignored and bind-mounted from
the host so they survive the container and can be copied out.

## The label is free, and that is the point

Each frame is labelled with the target's position in the camera's optical
frame, taken from **simulator ground truth**. Exact, instant, and
available for every frame.

The same dataset gathered on a real robot needs either a motion-capture
rig or a human drawing boxes, and a human is slower, more expensive and
less accurate than the number Gazebo already knows.

The cost is that everything learned from it is learned about a
*simulator*. That is why every manifest records `labels.source:
simulator` rather than leaving it to be assumed, and why the validator
rejects a dataset that does not say.

It also gives Phase 14 something honest to aim at: the geometric
detector from Phase 12 already solves this task to **30 mm**, measured.
A model that cannot beat that has not earned its place.

## A dataset is a claim, not a folder

Six months on, the question is never "what is in this folder" — it's
"was this recorded before or after the camera was re-aimed", and nothing
in the pixels answers that.

Every manifest field exists because its absence makes a specific question
unanswerable:

| field | without it |
|---|---|
| `code` | a model cannot be tied to the software that made its data |
| `world` | the target's colour, size and position are unknown |
| `robot` | camera height, pitch and FOV are unknown — labels are geometrically void while the images still look fine |
| `camera` | intrinsics a model implicitly learned, and that inference must match |
| `labels.source` | exact, annotated or computed — mixing them silently ruins a benchmark |

`provenance` is the idea the robot profiles have carried since Phase 1,
applied to data instead of masses.

### The container has git but no repository

`.git` is deliberately not mounted, so the recorder cannot resolve its
own commit. The first real recording produced a dataset with **no
provenance at all** — and the validator caught it, which is the only
reason it was noticed before the data was used.

The host resolves the commit and passes it in. A dirty tree is *recorded*,
not refused: refusing would make the tool unusable exactly while
developing the tool, and a dataset honestly labelled dirty beats one that
lies by omission.

## Two bugs the tooling found in its own data

### 96 files, 8 pictures

The first recorder drove to a pose, **stopped**, and took twelve frames.
With a stationary robot and a noiseless camera those twelve frames were
byte-identical.

The dataset advertised 96 samples and held 8 distinct images. Nothing
about the directory said so — the file count, the label count and the
manifest were all perfectly consistent, and every number computed
downstream was twelve times too optimistic.

`duplicate_report` hashes the files and now reports `distinct` alongside
the file count. A file count is not a sample count.

### Capturing while moving corrupts the label

The obvious fix — record continuously while driving — introduced a worse
bug. The label comes from ground truth read by asking Gazebo *after* the
image arrived, so image and label are from **different instants**. At
0.08 m/s, half a second of skew is 40 mm of label error: larger than the
30 mm the geometric baseline already achieves, which would make the
dataset worse than useless for improving on it.

The recorder now **steps and shoots**: a small move, a pause, a frame.
Each frame is a genuinely distinct viewpoint, and image and label agree.

A third trap, familiar from Phase 10: **record against a freshly started
simulator**. A recording attempt against a sim that earlier runs had
driven around dropped all 96 frames, every one as "target behind the
camera". The rejection accounting said so immediately; without it, a
recorder that keeps 8 of 96 frames looks exactly like one that worked.

## Splitting: the part that matters

A robot dataset is a **time series**, not a bag of independent samples.
Consecutive frames are near-duplicates with near-identical labels.
Shuffling frames and dealing them into train and test therefore puts
near-duplicates on both sides, and a model scored that way reports an
accuracy it will never reproduce.

Both splitters ship, and `make dataset-describe` runs both on the same
frames:

| split | nearest train neighbour (min) | median | within 10 mm |
|---|---|---|---|
| **by episode** (correct) | 27.4 mm | 51.1 mm | **0 of 12** |
| by frame (leaks) | 3.7 mm | 8.4 mm | **12 of 15** |

Four fifths of the test set sits within a centimetre of something the
model trained on. A test frame whose nearest training neighbour is
millimetres away is not a test — it's a frame the model has effectively
already seen, scored as though it hadn't.

`split_by_frame` is kept deliberately. It's what a tutorial reaches for
and what `train_test_split` does by default, and being able to *run* it
and measure the damage is worth more than a warning in a comment.

The correct split costs something honest: with few episodes it's coarse
and the ratios can't be hit exactly. That's a real limitation of having
recorded few episodes. Rounding it away by splitting frames doesn't fix
the data, it hides the problem — so `split_by_episode` **refuses** to run
on fewer than three episodes rather than falling back.

## Why not rosbag2

A bag is a replayable **log**; this is a derived **dataset**. Measured on
this camera:

| | per frame |
|---|---|
| rosbag2 | 922 kB — exactly 640×480×3, raw and uncompressed |
| PNG | 11 kB |

**87x**, and the bag still has to be extracted before anything can train
on it, with labels attached during extraction anyway.

The 11 kB deserves a caveat: PNG compresses a synthetic scene of flat
colours extremely well, and a real camera would be an order of magnitude
larger. 87x is the honest number for *this* setup, not a general claim.

Record bags when the question is "what happened". This answers "what
should the model see".

## What this dataset is not

- **96 frames across 8 episodes is small.** It is enough to build the
  pipeline and demonstrate the split, and not enough to train anything
  that generalises. Phase 14 must either record much more or say plainly
  what it is fitting.
- **One target, one colour, one world.** No occlusion, no clutter, no
  lighting variation, no distractors of the same hue.
- **No sensor noise.** Inherited from Phase 12, where it is deliberate.
- **Simulation only.** Nothing here has seen a real camera.

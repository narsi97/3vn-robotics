# Copyright 2026 3VN Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Reading a dataset, and splitting it without lying to yourself.

The splitting is the whole reason this module exists.

A robot dataset is a TIME SERIES, not a bag of independent samples. At
15 Hz the robot moves about a centimetre between frames, so consecutive
frames are very nearly the same picture with very nearly the same label.
Shuffling frames and dealing them into train and test therefore puts
near-duplicates on both sides, and a model scored that way reports an
accuracy it will never reproduce on a robot that has actually moved.

This is not a subtle effect and it is not hypothetical: `leakage_report`
measures it on the real recording, and the numbers are in
docs/data-pipeline.md.

Both splitters are provided. `split_by_frame` is here precisely because
it is the wrong one - it is what a tutorial reaches for, it is what
sklearn's train_test_split does by default, and being able to run it and
measure the damage is worth more than a warning in a comment.
"""

import collections
import hashlib
import json
import pathlib
import random

import numpy as np

#: One labelled example.
Frame = collections.namedtuple('Frame', 'image episode index label stamp')


def load(directory):
    """
    Read every labelled frame from a dataset directory.

    Labels live in a JSONL file rather than one JSON per image: a
    thousand tiny files is slow to read, awkward to diff and easy to
    half-delete, while one line per frame stays greppable and appends
    cheaply while recording.
    """
    directory = pathlib.Path(directory)
    labels = directory / 'labels.jsonl'
    if not labels.is_file():
        raise FileNotFoundError(f'{labels} does not exist')

    frames = []
    for line in labels.read_text().splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        frames.append(Frame(
            image=directory / row['image'],
            episode=row['episode'],
            index=row['index'],
            label=tuple(row['label']),
            stamp=row['stamp'],
        ))
    return frames


def episodes(frames):
    """Return the episode ids present, in order."""
    return sorted({frame.episode for frame in frames})


def split_by_episode(frames, ratios=(0.7, 0.15, 0.15), seed=0):
    """
    Split so that no episode appears in more than one part.

    THE CORRECT SPLIT FOR THIS DATA.

    An episode is one continuous run: the robot drives, the camera sees a
    smoothly changing scene, and every frame in it is highly correlated
    with its neighbours. Keeping an episode whole means the test set
    contains situations the model genuinely has not seen, which is the
    only thing a test set is for.

    It also costs something honest: with few episodes the split is
    coarse, and the ratios cannot be hit exactly. That is a real
    limitation of having recorded few episodes, and rounding it away by
    splitting frames instead does not fix the data, it only hides the
    problem.
    """
    ids = episodes(frames)
    if len(ids) < 3:
        raise ValueError(
            f'only {len(ids)} episode(s): an episode-wise split into three '
            f'parts needs at least three. Record more episodes rather than '
            f'falling back to a frame split, which would leak.')

    shuffled = list(ids)
    random.Random(seed).shuffle(shuffled)

    n_train = max(1, round(len(shuffled) * ratios[0]))
    n_val = max(1, round(len(shuffled) * ratios[1]))
    # Whatever is left is test, so no episode is ever dropped by rounding.
    n_train = min(n_train, len(shuffled) - 2)
    n_val = min(n_val, len(shuffled) - n_train - 1)

    assignment = {}
    for position, episode in enumerate(shuffled):
        if position < n_train:
            assignment[episode] = 'train'
        elif position < n_train + n_val:
            assignment[episode] = 'val'
        else:
            assignment[episode] = 'test'

    out = {'train': [], 'val': [], 'test': []}
    for frame in frames:
        out[assignment[frame.episode]].append(frame)
    return out


def split_by_frame(frames, ratios=(0.7, 0.15, 0.15), seed=0):
    """
    Shuffle every frame and deal them out. THE WRONG SPLIT. Provided on
    purpose.

    This is what a tutorial reaches for and what `train_test_split` does
    by default. On independent samples it is right; on a time series it
    puts frame 41 in train and frame 42 in test, and those two pictures
    differ by about a centimetre of robot motion.

    Keeping it here, runnable, is what lets `leakage_report` put a number
    on the damage instead of asserting that there is some.
    """
    shuffled = list(frames)
    random.Random(seed).shuffle(shuffled)
    n_train = int(len(shuffled) * ratios[0])
    n_val = int(len(shuffled) * ratios[1])
    return {
        'train': shuffled[:n_train],
        'val': shuffled[n_train:n_train + n_val],
        'test': shuffled[n_train + n_val:],
    }


def leakage_report(split):
    """
    Measure how close each test frame's nearest training frame is.

    Distance is between LABELS, in metres, which is the honest proxy
    here: two frames whose target sits within a few millimetres of the
    same place are near-duplicate problems whatever their pixels do.

    Returns the minimum, median and the count of test frames whose
    nearest training neighbour is closer than a centimetre. On a correct
    split those numbers describe genuinely unseen situations; on a
    leaking one the minimum approaches zero and the count approaches the
    size of the test set.
    """
    train = np.array([f.label for f in split['train']], dtype=float)
    test = np.array([f.label for f in split['test']], dtype=float)
    if not len(train) or not len(test):
        raise ValueError('both train and test must be non-empty')

    # (n_test, n_train) pairwise distances.
    deltas = test[:, None, :] - train[None, :, :]
    distances = np.linalg.norm(deltas, axis=2)
    nearest = distances.min(axis=1)

    return {
        'test_frames': int(len(test)),
        'min_mm': float(nearest.min() * 1000.0),
        'median_mm': float(np.median(nearest) * 1000.0),
        'within_10mm': int((nearest < 0.010).sum()),
        'fraction_within_10mm': float((nearest < 0.010).mean()),
    }


def summarise(frames):
    """Return counts and label ranges, for a quick look at coverage."""
    labels = np.array([f.label for f in frames], dtype=float)
    per_episode = collections.Counter(f.episode for f in frames)
    return {
        'frames': len(frames),
        'episodes': len(per_episode),
        'frames_per_episode': dict(sorted(per_episode.items())),
        'label_min': labels.min(axis=0).tolist(),
        'label_max': labels.max(axis=0).tolist(),
        'label_mean': labels.mean(axis=0).tolist(),
    }


def duplicate_report(frames):
    """
    Count how many of the images are actually distinct.

    A file count is not a sample count. The first recording made here
    drove to a pose, stopped, and took twelve frames: with a stationary
    robot and a noiseless camera those twelve were byte-identical, so a
    dataset advertising 96 samples held 8 pictures. Everything downstream
    - the split, the epoch size, the reported accuracy - was computed
    against a number that was twelve times too large.

    Hashing whole files catches exactly that case. It does NOT catch
    near-duplicates, which are the subtler and more common problem, and
    that is what `leakage_report` is for. The two are complementary:
    this one finds a broken recorder, that one finds a broken split.
    """
    digests = {}
    for frame in frames:
        digests[frame.image.name] = hashlib.sha256(
            frame.image.read_bytes()).hexdigest()

    counts = collections.Counter(digests.values())
    repeated = {digest: n for digest, n in counts.items() if n > 1}
    per_episode = collections.defaultdict(set)
    for frame in frames:
        per_episode[frame.episode].add(digests[frame.image.name])

    return {
        'files': len(digests),
        'unique': len(counts),
        'duplicate_groups': len(repeated),
        'largest_group': max(counts.values()) if counts else 0,
        'unique_per_episode': {k: len(v) for k, v in sorted(per_episode.items())},
    }

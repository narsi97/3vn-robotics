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
Splitting a time series without lying to yourself.

The dataset built here mimics the real one: several episodes, each a
smooth trajectory whose consecutive frames differ by millimetres. That
structure is what makes a frame-wise split dishonest, so it has to be
present for the tests to mean anything.
"""

import json

import pytest
from threevn_data import dataset as ds

EPISODES = 8
PER_EPISODE = 12
#: Metres between consecutive frames, matched to the real recording:
#: its frame-wise split found nearest neighbours 3.7 mm apart with a
#: median of 8.4 mm. The value matters, because it has to fall BELOW the
#: 10 mm threshold leakage_report counts against - which is the whole
#: point. An earlier 20 mm here put consecutive frames further apart than
#: the threshold and made the leak invisible.
STEP = 0.006

#: Metres between one episode's starting point and the next. It must
#: exceed the distance an episode covers (PER_EPISODE * STEP), or the
#: episodes overlap in label space and an episode-wise split has no gap
#: to offer - which is exactly what the first version of this fixture
#: did, making the leakage test fail for a reason that was about the
#: fixture rather than the splitter. The real recorder repositions
#: between episodes for the same reason.
EPISODE_SPACING = 0.40


@pytest.fixture
def recorded(tmp_path):
    """
    Write a synthetic dataset with realistic temporal structure.

    Each episode walks away from a different starting point in small
    steps, so neighbouring frames are near-duplicates and distant
    episodes are genuinely different.
    """
    images = tmp_path / 'images'
    images.mkdir()
    rows = []
    for episode in range(EPISODES):
        base = 0.30 + episode * EPISODE_SPACING
        for index in range(PER_EPISODE):
            name = f'ep{episode:03d}_{index:04d}.png'
            # Distinct bytes per frame: a dataset of identical images is
            # its own bug, tested separately below.
            (images / name).write_bytes(f'{episode}:{index}'.encode())
            rows.append({
                'image': f'images/{name}',
                'episode': episode,
                'index': index,
                'label': [base + index * STEP, 0.01 * episode, 0.35],
                'stamp': float(episode * 100 + index),
            })
    (tmp_path / 'labels.jsonl').write_text(
        '\n'.join(json.dumps(r) for r in rows) + '\n')
    return tmp_path


def test_every_frame_is_loaded(recorded):
    """Load returns one Frame per line, with the label parsed."""
    frames = ds.load(recorded)
    assert len(frames) == EPISODES * PER_EPISODE
    assert len(frames[0].label) == 3
    assert frames[0].image.is_file()


def test_a_missing_label_file_is_an_error(tmp_path):
    """A directory of images with no labels is not a dataset."""
    with pytest.raises(FileNotFoundError):
        ds.load(tmp_path)


# -- the split ----------------------------------------------------------

def test_no_episode_appears_in_two_splits(recorded):
    """
    THE PROPERTY THE CORRECT SPLIT EXISTS TO GUARANTEE.

    An episode is one continuous run. Splitting it across train and test
    puts the same situation on both sides of the evaluation.
    """
    split = ds.split_by_episode(ds.load(recorded))
    seen = {}
    for name, frames in split.items():
        for frame in frames:
            assert seen.setdefault(frame.episode, name) == name, (
                f'episode {frame.episode} appears in more than one split'
            )


def test_the_split_keeps_every_frame(recorded):
    """Nothing is dropped by rounding."""
    frames = ds.load(recorded)
    split = ds.split_by_episode(frames)
    assert sum(len(v) for v in split.values()) == len(frames)


def test_all_three_splits_are_populated(recorded):
    """A test set of zero frames reports a perfect score."""
    split = ds.split_by_episode(ds.load(recorded))
    for name, frames in split.items():
        assert frames, f'{name} is empty'


def test_the_split_is_deterministic(recorded):
    """
    Same seed, same split.

    Two runs that disagree make every comparison between them
    meaningless, and the disagreement is invisible in the numbers.
    """
    frames = ds.load(recorded)
    a = ds.split_by_episode(frames, seed=7)
    b = ds.split_by_episode(frames, seed=7)
    assert [f.image for f in a['test']] == [f.image for f in b['test']]


def test_too_few_episodes_is_refused_not_worked_around(tmp_path):
    """
    With two episodes there is no honest three-way split.

    Falling back to a frame split would produce numbers, and they would
    be wrong. The error says to record more episodes.
    """
    images = tmp_path / 'images'
    images.mkdir()
    rows = []
    for episode in range(2):
        for index in range(5):
            name = f'ep{episode}_{index}.png'
            (images / name).write_bytes(b'x')
            rows.append({'image': f'images/{name}', 'episode': episode,
                         'index': index, 'label': [0.3, 0.0, 0.35],
                         'stamp': 0.0})
    (tmp_path / 'labels.jsonl').write_text(
        '\n'.join(json.dumps(r) for r in rows) + '\n')

    with pytest.raises(ValueError, match='at least three'):
        ds.split_by_episode(ds.load(tmp_path))


# -- leakage ------------------------------------------------------------

def test_the_frame_split_leaks_and_the_episode_split_does_not(recorded):
    """
    THE MEASUREMENT THAT JUSTIFIES THE WHOLE MODULE.

    Both splitters run on the same frames. The frame-wise one puts
    neighbouring frames of one trajectory on opposite sides, so the
    nearest training example to a test frame is one step away. The
    episode-wise one leaves a real gap.

    On the actual recording this is 8.4 mm median and 12 of 15 test
    frames within a centimetre, against 51.1 mm and none.
    """
    frames = ds.load(recorded)
    leaky = ds.leakage_report(ds.split_by_frame(frames))
    clean = ds.leakage_report(ds.split_by_episode(frames))

    assert leaky['min_mm'] <= STEP * 1000 * 1.5, (
        'a frame split should leave a test frame within one step of a '
        'training frame'
    )
    assert clean['min_mm'] > leaky['min_mm'] * 2, (
        f'the episode split must put real distance between train and test: '
        f'got {clean["min_mm"]:.1f} mm against a leaking {leaky["min_mm"]:.1f} mm'
    )
    assert clean['fraction_within_10mm'] < leaky['fraction_within_10mm']


def test_leakage_needs_both_sides(recorded):
    """An empty split cannot be scored, and saying so beats returning zero."""
    split = ds.split_by_episode(ds.load(recorded))
    split['test'] = []
    with pytest.raises(ValueError):
        ds.leakage_report(split)


# -- duplicates ---------------------------------------------------------

def test_distinct_frames_are_reported_as_distinct(recorded):
    """The healthy case, so the failing one below is meaningful."""
    report = ds.duplicate_report(ds.load(recorded))
    assert report['unique'] == report['files'] == EPISODES * PER_EPISODE
    assert report['duplicate_groups'] == 0


def test_identical_images_are_caught(recorded):
    """
    A FILE COUNT IS NOT A SAMPLE COUNT.

    The first real recording drove to a pose, stopped, and took twelve
    frames. With a stationary robot and a noiseless camera those twelve
    were byte-identical: 96 files, 8 pictures. Nothing about the
    directory said so, and every number computed downstream was twelve
    times too optimistic.
    """
    for path in (recorded / 'images').glob('*.png'):
        path.write_bytes(b'the same picture every time')

    report = ds.duplicate_report(ds.load(recorded))
    assert report['unique'] == 1
    assert report['largest_group'] == EPISODES * PER_EPISODE
    assert all(count == 1
               for count in report['unique_per_episode'].values())


def test_summarise_reports_coverage(recorded):
    """Counts and label ranges, for seeing at a glance what was recorded."""
    summary = ds.summarise(ds.load(recorded))
    assert summary['frames'] == EPISODES * PER_EPISODE
    assert summary['episodes'] == EPISODES
    assert summary['label_min'][0] < summary['label_max'][0]

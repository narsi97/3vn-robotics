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

"""What a dataset must say about itself before it can be believed."""

import json

import pytest
from threevn_data import manifest as m

CAMERA = {'fx': 532.57, 'fy': 532.57, 'cx': 320.0, 'cy': 240.0,
          'width': 640, 'height': 480, 'frame_id': 'arm_camera_optical_frame'}
LABELS = {'source': 'simulator', 'quantity': 'target position', 'units': 'm'}


@pytest.fixture
def good(monkeypatch):
    """Provide a manifest that validates."""
    monkeypatch.setenv('THREEVN_GIT_COMMIT', 'a' * 40)
    monkeypatch.setenv('THREEVN_GIT_DIRTY', '0')
    return m.build(world='bench_with_target', robot_profile='threevn_mm_v1',
                   camera=CAMERA, label_source=LABELS, episodes=8, frames=96)


def test_a_complete_manifest_validates(good):
    """The happy path, so the failures below mean something."""
    assert m.validate(good) == []


def test_the_commit_comes_from_the_environment(good):
    """
    The recorder runs in a container with git but no repository.

    .git is deliberately not mounted, so the host resolves the commit and
    passes it in. Without this the first real recording produced a
    dataset with no provenance at all, which the validator caught.
    """
    assert good['code']['commit'] == 'a' * 40
    assert good['code']['dirty'] is False


def test_a_dirty_tree_is_recorded_not_refused(monkeypatch):
    """
    Refusing would make the tool unusable while developing the tool.

    A dataset honestly labelled dirty is far more useful than one that
    lies by omission.
    """
    monkeypatch.setenv('THREEVN_GIT_COMMIT', 'b' * 40)
    monkeypatch.setenv('THREEVN_GIT_DIRTY', '1')
    doc = m.build(world='w', robot_profile='r', camera=CAMERA,
                  label_source=LABELS, episodes=1, frames=1)
    assert doc['code']['dirty'] is True
    assert m.validate(doc) == []


def test_a_manifest_without_a_commit_is_rejected(good):
    """Data that cannot be tied to code cannot be reproduced."""
    good['code'] = {'commit': None, 'dirty': None}
    assert any('commit' in p for p in m.validate(good))


def test_every_required_field_is_required(good):
    """Drop each field in turn; each one must be missed."""
    for field in m.REQUIRED:
        broken = dict(good)
        broken.pop(field)
        assert m.validate(broken), f'removing {field!r} was not noticed'


@pytest.mark.parametrize('source', ['guessed', '', None, 'auto'])
def test_the_label_source_must_be_one_of_the_known_kinds(good, source):
    """
    Exact, annotated or computed. Mixing them silently ruins a benchmark.

    Simulator ground truth is free and exact; a human annotator is
    neither. A dataset that does not say which it used cannot be
    compared with one that did.
    """
    good['labels'] = dict(LABELS, source=source)
    assert any('labels.source' in p for p in m.validate(good))


def test_an_empty_dataset_is_not_a_dataset(good):
    """Zero frames validates as a problem, not as a small dataset."""
    good['frames'] = 0
    assert any('no frames' in p for p in m.validate(good))


def test_missing_intrinsics_are_caught(good):
    """
    Intrinsics are what a model implicitly learns.

    Anything deprojecting a pixel at inference time has to match them,
    and a dataset that does not record them cannot be checked against
    the camera that later reads it.
    """
    good['camera'] = {'width': 640, 'height': 480}
    problems = m.validate(good)
    assert any("'fx'" in p for p in problems)


def test_a_future_schema_is_refused(good):
    """A reader must not pretend to understand a layout it has not seen."""
    good['schema_version'] = m.SCHEMA_VERSION + 1
    assert any('schema_version' in p for p in m.validate(good))


def test_round_trips_through_disk(tmp_path, good):
    """Written and read back unchanged."""
    m.write(tmp_path, good)
    assert m.read(tmp_path) == good


def test_reading_a_directory_without_a_manifest_says_why(tmp_path):
    """
    The error explains what is wrong with the DIRECTORY, not the call.

    'manifest.json not found' invites someone to write an empty one. The
    message says why a directory of images is not a dataset.
    """
    with pytest.raises(FileNotFoundError, match='is not a dataset'):
        m.read(tmp_path)


def test_the_content_hash_notices_a_rename(tmp_path):
    """
    Same bytes under a different name is a different dataset.

    Labels are paired to images by filename, so a rename that loses the
    pairing must change the hash even though no pixel moved.
    """
    (tmp_path / 'a.png').write_bytes(b'one')
    (tmp_path / 'b.png').write_bytes(b'two')
    before = m.content_hash(tmp_path.glob('*.png'))

    (tmp_path / 'a.png').rename(tmp_path / 'c.png')
    after = m.content_hash(tmp_path.glob('*.png'))
    assert before != after


def test_the_manifest_is_written_deterministically(tmp_path, good):
    """Sorted keys, so a diff of two manifests shows what actually changed."""
    m.write(tmp_path, good)
    text = (tmp_path / 'manifest.json').read_text()
    assert text == json.dumps(good, indent=2, sort_keys=True) + '\n'

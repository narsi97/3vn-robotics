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
What a dataset claims about itself.

A dataset is not a folder of images. It is a folder of images plus a
claim about where they came from, and the claim is the part that rots.
Six months on, the question is never "what is in this folder" - it is
"was this recorded before or after the camera was re-aimed", and nothing
in the pixels answers that.

Every field here exists because its absence makes a specific question
unanswerable:

    code        which commit recorded it. Without this, a model trained
                on the data cannot be tied to the software that made it,
                and Phase 15 has nothing to put in a registry.
    world       which scene. The target's colour, size and position are
                properties of the world file, so a dataset recorded
                against a different one is a different dataset wearing
                the same name.
    robot       which profile. Camera height, pitch and field of view
                all come from here; re-aiming the camera invalidates
                every label geometrically while leaving the images
                looking fine.
    camera      the intrinsics AS PUBLISHED, not as configured. These
                are what a model implicitly learns, and they are what
                any deprojection at inference time must match.
    labels      where the truth came from. Simulator ground truth is
                exact and free; a human annotator is neither. Mixing the
                two silently is how a benchmark stops meaning anything.

`provenance` is the same idea the robot profiles have carried since
Phase 1, applied to data instead of masses.
"""

import datetime
import hashlib
import json
import os
import pathlib
import subprocess

#: Bumped when the on-disk layout changes in a way that older readers
#: cannot handle. A dataset that does not declare one predates the idea
#: and is not readable.
SCHEMA_VERSION = 1

REQUIRED = ('schema_version', 'created', 'code', 'world', 'robot',
            'camera', 'labels', 'episodes', 'frames')


def git_description(repo=None):
    """
    Return the commit the recorder is running from, and whether it was dirty.

    A dirty tree is recorded rather than refused. Refusing would make the
    tool unusable exactly when it is most useful - while developing the
    recorder itself - and a dataset honestly labelled `dirty` is far more
    useful than one that lies by omission.
    """
    # The recorder runs inside a container that has git but no
    # repository, because .git is deliberately not mounted. The host
    # resolves the commit and passes it in; falling back to running git
    # keeps this usable anywhere else.
    commit = os.environ.get('THREEVN_GIT_COMMIT')
    if commit:
        return {'commit': commit,
                'dirty': os.environ.get('THREEVN_GIT_DIRTY') == '1'}

    repo = pathlib.Path(repo) if repo else pathlib.Path.cwd()
    try:
        sha = subprocess.run(
            ['git', '-C', str(repo), 'rev-parse', 'HEAD'],
            capture_output=True, text=True, timeout=10, check=True).stdout.strip()
        status = subprocess.run(
            ['git', '-C', str(repo), 'status', '--porcelain'],
            capture_output=True, text=True, timeout=10, check=True).stdout.strip()
    except (subprocess.SubprocessError, OSError):
        return {'commit': None, 'dirty': None}
    return {'commit': sha, 'dirty': bool(status)}


def build(world, robot_profile, camera, label_source, episodes, frames,
          repo=None, extra=None):
    """Assemble a manifest describing one recording session."""
    manifest = {
        'schema_version': SCHEMA_VERSION,
        'created': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'code': git_description(repo),
        'world': world,
        'robot': robot_profile,
        'camera': dict(camera),
        'labels': dict(label_source),
        'episodes': int(episodes),
        'frames': int(frames),
    }
    if extra:
        manifest.update(extra)
    return manifest


def validate(manifest):
    """
    Return a list of problems with a manifest. Empty means usable.

    Returns rather than raises: a caller loading twenty datasets wants to
    know which are broken, not to stop at the first.
    """
    problems = []

    missing = [key for key in REQUIRED if key not in manifest]
    if missing:
        problems.append(f'missing required fields: {sorted(missing)}')
        return problems

    if manifest['schema_version'] != SCHEMA_VERSION:
        problems.append(
            f'schema_version {manifest["schema_version"]} is not '
            f'{SCHEMA_VERSION}; this reader cannot promise to understand it')

    if not manifest['code'].get('commit'):
        problems.append(
            'no commit recorded: this dataset cannot be tied to the code '
            'that produced it')

    for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height'):
        if key not in manifest['camera']:
            problems.append(f'camera is missing {key!r}')

    if manifest['labels'].get('source') not in ('simulator', 'human', 'derived'):
        problems.append(
            f'labels.source is {manifest["labels"].get("source")!r}; it must '
            f'say whether the truth is exact (simulator), annotated (human) '
            f'or computed from something else (derived)')

    if manifest['frames'] <= 0:
        problems.append('a dataset with no frames is not a dataset')
    if manifest['episodes'] <= 0:
        problems.append('frames must belong to at least one episode')

    return problems


def content_hash(paths):
    """
    Return a stable hash over a set of files.

    Sorted, and the NAME is hashed alongside the bytes: two datasets with
    the same images under different names are not the same dataset, and a
    rename that loses the pairing with the labels must change the hash.
    """
    digest = hashlib.sha256()
    for path in sorted(pathlib.Path(p) for p in paths):
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def write(directory, manifest):
    """Write the manifest beside the data it describes."""
    path = pathlib.Path(directory) / 'manifest.json'
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    return path


def read(directory):
    """Read a manifest, or raise with a message that says what to do."""
    path = pathlib.Path(directory) / 'manifest.json'
    if not path.is_file():
        raise FileNotFoundError(
            f'{path} does not exist. A directory of images with no manifest '
            f'is not a dataset: nothing records which world, robot or code '
            f'produced it, so no result from it can be reproduced.')
    return json.loads(path.read_text())

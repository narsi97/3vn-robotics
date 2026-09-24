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
Where models live, and which one the robot is running.

A directory of files and one pointer. No database, no service, no
daemon: the whole state is readable with `cat`, survives the container,
and can be copied to a robot with `scp`. Anything more would be
infrastructure this project does not have a problem for.

The design borrows what actually matters from a real registry:

    versions are IMMUTABLE       a promoted model cannot change under a
                                 running robot. Re-registering the same
                                 bytes returns the existing version
                                 rather than making a second one
    production is a POINTER      promotion and rollback are the same
                                 operation with different arguments, so
                                 rollback is as well-tested as promotion
    history is APPEND-ONLY       "what was running last Tuesday" is a
                                 question the log answers, and it is the
                                 first question asked after a failure

Versions are content-addressed. A version number would need a counter,
and a counter is state that can disagree with the files; a hash of the
artifact cannot.
"""

import datetime
import hashlib
import json
import pathlib

INDEX = 'index.json'
MODELS = 'models'


class Registry:
    """A directory of versioned models with a production pointer."""

    def __init__(self, root):
        self.root = pathlib.Path(root)
        (self.root / MODELS).mkdir(parents=True, exist_ok=True)

    # -- state ---------------------------------------------------------

    @property
    def index_path(self):
        """Path to the index file."""
        return self.root / INDEX

    def _read_index(self):
        if not self.index_path.is_file():
            return {'production': None, 'versions': {}, 'history': []}
        return json.loads(self.index_path.read_text())

    def _write_index(self, index):
        self.index_path.write_text(
            json.dumps(index, indent=2, sort_keys=True) + '\n')

    # -- registering ---------------------------------------------------

    @staticmethod
    def version_of(artifact):
        """
        Return the content-addressed version id for an artifact.

        The metrics and provenance are part of the identity, not just the
        weights: the same weights fitted on different data are a
        different model to anyone trying to explain what the robot did.
        """
        payload = json.dumps(artifact, sort_keys=True).encode()
        return hashlib.sha256(payload).hexdigest()[:12]

    def register(self, artifact, note=''):
        """
        Store an artifact and return its version.

        Idempotent. Registering identical bytes twice returns the same
        version and does not append to the log, because nothing changed
        and a log full of no-ops is a log nobody reads.
        """
        version = self.version_of(artifact)
        index = self._read_index()
        if version in index['versions']:
            return version

        path = self.root / MODELS / f'{version}.json'
        path.write_text(json.dumps(artifact, indent=2, sort_keys=True) + '\n')

        metrics = artifact.get('metrics', {})
        index['versions'][version] = {
            'registered': _now(),
            'note': note,
            'file': f'{MODELS}/{version}.json',
            'test_median_mm': _score(artifact),
            'baseline_median_mm': _baseline(metrics),
            'dataset_frames': artifact.get('dataset', {}).get('frames'),
            'code': artifact.get('code', {}).get('commit'),
        }
        index['history'].append(
            {'at': _now(), 'action': 'register', 'version': version,
             'note': note})
        self._write_index(index)
        return version

    # -- reading -------------------------------------------------------

    def versions(self):
        """Return {version: summary}, newest first."""
        index = self._read_index()
        return dict(sorted(index['versions'].items(),
                           key=lambda kv: kv[1]['registered'], reverse=True))

    def artifact(self, version):
        """Load one stored artifact."""
        path = self.root / MODELS / f'{version}.json'
        if not path.is_file():
            raise FileNotFoundError(f'no model {version!r} in {self.root}')
        return json.loads(path.read_text())

    def production_version(self):
        """Return the version the robot should be running, or None."""
        return self._read_index()['production']

    def production(self):
        """Load the production artifact, or None if nothing is promoted."""
        version = self.production_version()
        return self.artifact(version) if version else None

    def history(self):
        """Return the append-only log, oldest first."""
        return list(self._read_index()['history'])

    # -- promoting -----------------------------------------------------

    def promote(self, version, reason='', forced=False):
        """
        Point production at a version.

        The gates are NOT run here. Deciding and recording are separate
        on purpose: the caller runs the gates, and a forced promotion is
        recorded as forced rather than being indistinguishable from one
        that passed.
        """
        index = self._read_index()
        if version not in index['versions']:
            raise KeyError(f'{version!r} is not registered')

        previous = index['production']
        if previous == version:
            # Idempotent, and not merely for tidiness. Appending a
            # promotion whose `from` is the version itself makes the
            # next rollback a no-op: it would dutifully restore what is
            # already running, which is the one moment that failing
            # quietly is least acceptable.
            return previous
        index['production'] = version
        index['history'].append({
            'at': _now(), 'action': 'promote', 'version': version,
            'from': previous, 'reason': reason, 'forced': bool(forced),
        })
        self._write_index(index)
        return previous

    def rollback(self, reason=''):
        """
        Put back whatever was in production before the last promotion.

        The same operation as promote, which is the point: rollback is
        the path taken when something is already wrong, and a rollback
        with its own separate machinery is one tested on the day it is
        first needed.
        """
        index = self._read_index()
        promotions = [row for row in index['history']
                      if row['action'] == 'promote']
        if not promotions:
            raise RuntimeError('nothing has ever been promoted')
        target = promotions[-1]['from']
        if target is None:
            raise RuntimeError(
                'the current model is the first ever promoted; there is '
                'nothing to roll back to. Deploying a first model is a '
                'step forward with no undo, which is worth knowing before '
                'rather than after')
        return self.promote(target, reason=reason or 'rollback')


def _now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _score(artifact):
    """
    Return the stored model's test median, by NAME not by guess.

    See gates.selected_key, which is the single answer to which metrics
    entry a stored model corresponds to.
    """
    from threevn_mlops.gates import selected_key

    key = selected_key(artifact)
    if key is None:
        return None
    return artifact['metrics'][key].get('test', {}).get('median_mm')


def _baseline(metrics):
    return metrics.get('geometric', {}).get('test', {}).get('median_mm')

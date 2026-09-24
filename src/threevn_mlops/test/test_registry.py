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

"""Storing models, pointing at one, and putting the previous one back."""

import pytest
from threevn_mlops.registry import Registry


@pytest.fixture
def registry(tmp_path):
    """An empty registry."""
    return Registry(tmp_path / 'registry')


def test_a_version_is_the_content(registry, good, make_artifact):
    """
    Content-addressed, so the id cannot disagree with the file.

    A counter is state, and state can drift from what it counts.
    """
    first = registry.register(good)
    same = registry.register(make_artifact())
    assert first == same, 'identical artifacts are one version'

    different = registry.register(make_artifact(test_median=9.0))
    assert different != first


def test_registering_twice_does_not_grow_the_log(registry, good, make_artifact):
    """A log full of no-ops is a log nobody reads."""
    registry.register(good)
    before = len(registry.history())
    registry.register(make_artifact())
    assert len(registry.history()) == before


def test_metrics_and_provenance_are_recorded_where_they_can_be_read(
        registry, good):
    """
    `status` has to answer without opening every artifact.

    The score summarised here is the SELECTED model's, by name.
    """
    version = registry.register(good, note='first')
    row = registry.versions()[version]
    assert row['test_median_mm'] == pytest.approx(10.5)
    assert row['baseline_median_mm'] == pytest.approx(106.8)
    assert row['dataset_frames'] == 480
    assert row['note'] == 'first'


def test_nothing_is_in_production_until_promoted(registry, good):
    """
    A registry with models and no pointer serves nothing.

    The serving node refuses to start rather than picking one, because
    picking one is exactly the untracked deployment this prevents.
    """
    registry.register(good)
    assert registry.production_version() is None
    assert registry.production() is None


def test_promotion_moves_the_pointer_and_records_where_from(registry,
                                                            make_artifact):
    """History answers "what was running last Tuesday"."""
    first = registry.register(make_artifact(test_median=20.0))
    second = registry.register(make_artifact(test_median=10.0))

    assert registry.promote(first) is None
    assert registry.promote(second) == first
    assert registry.production_version() == second

    promotions = [r for r in registry.history() if r['action'] == 'promote']
    assert promotions[-1]['from'] == first


def test_promoting_what_is_already_running_changes_nothing(registry, good):
    """
    IDEMPOTENT, AND NOT MERELY FOR TIDINESS.

    Recording a promotion whose `from` is the version itself makes the
    next rollback a no-op: it restores what is already running, at the
    one moment when failing quietly is least acceptable.
    """
    version = registry.register(good)
    registry.promote(version)
    before = len(registry.history())

    registry.promote(version)
    assert len(registry.history()) == before
    assert registry.production_version() == version


def test_rollback_restores_the_previous_model(registry, make_artifact):
    """The path taken when something is already wrong."""
    good_one = registry.register(make_artifact(test_median=10.0))
    bad_one = registry.register(make_artifact(test_median=90.0))
    registry.promote(good_one)
    registry.promote(bad_one, reason='mistake')

    registry.rollback(reason='it was worse')
    assert registry.production_version() == good_one


def test_rollback_survives_a_repeated_promotion(registry, make_artifact):
    """
    The regression the idempotence fix exists to prevent.

    Promote A, promote A again, promote B. Rolling back must reach A,
    not bounce off a self-referential history entry.
    """
    a = registry.register(make_artifact(test_median=10.0))
    b = registry.register(make_artifact(test_median=90.0))
    registry.promote(a)
    registry.promote(a)
    registry.promote(b)

    registry.rollback()
    assert registry.production_version() == a


def test_rolling_back_the_first_ever_deployment_says_why(registry, good):
    """
    There is nothing behind it, and the message says so.

    Worth knowing before the first deployment rather than during it.
    """
    version = registry.register(good)
    registry.promote(version)
    with pytest.raises(RuntimeError, match='nothing to roll back to'):
        registry.rollback()


def test_rolling_back_with_no_history_is_refused(registry):
    """An empty registry cannot restore anything."""
    with pytest.raises(RuntimeError, match='ever been promoted'):
        registry.rollback()


def test_promoting_something_unregistered_is_refused(registry):
    """Serving a file nobody stored is the thing being prevented."""
    with pytest.raises(KeyError):
        registry.promote('deadbeefcafe')


def test_a_forced_promotion_is_recorded_as_forced(registry, good):
    """
    Distinguishable from one that passed.

    Otherwise the log says a model was promoted and omits that every
    gate objected, which is the single most useful fact about it.
    """
    version = registry.register(good)
    registry.promote(version, reason='shipping anyway', forced=True)
    assert registry.history()[-1]['forced'] is True


def test_stored_artifacts_are_readable_as_files(registry, good, tmp_path):
    """
    The whole state is `cat`-able.

    No database and no daemon: a registry that can be copied to a robot
    with scp and read on arrival.
    """
    version = registry.register(good)
    assert (registry.root / 'models' / f'{version}.json').is_file()
    assert registry.index_path.is_file()
    assert registry.artifact(version) == good

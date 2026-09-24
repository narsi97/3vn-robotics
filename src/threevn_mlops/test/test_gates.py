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
The conditions a model has to meet. Refusing is the feature.

Each test states what goes wrong on a robot if the gate is absent.
"""

from conftest import FEATURES
import pytest
from threevn_mlops import gates


def test_a_good_model_passes_everything(good):
    """The happy path, so the refusals below mean something."""
    assert gates.check_all(good, FEATURES) == {}


def test_the_selected_model_is_named_not_guessed(good):
    """
    THE ARTIFACT SAYS WHICH METRICS ENTRY IT SAVED.

    This decision was reimplemented three times - gates, registry
    summary, serving banner - and two got it wrong identically by
    taking the alphabetically last key. With an ablation recording
    'ridge geometry' and 'ridge + shape' that picks the wrong one,
    because '+' sorts before 'g', and reports a model scoring 10.5 mm
    as scoring 46.5. Nothing failed; the number was just wrong.
    """
    assert gates.selected_key(good) == 'ridge + shape'

    good.pop('selected')
    problems = gates.validates(good)
    assert any('selected' in p for p in problems)


def test_a_model_that_loses_to_the_geometry_is_refused(make_artifact):
    """
    IT WOULD BE STRICTLY WORSE THAN CODE NEEDING NO DATA.

    Phase 12 solves this in closed form. A model that cannot beat it has
    bought nothing and costs a dataset, a training run and a registry.
    """
    losing = make_artifact(test_median=120.0, baseline=106.8)
    reasons = gates.beats_baseline(losing)
    assert reasons and 'does not beat the closed form' in reasons[0]


def test_a_tie_with_the_baseline_is_refused(make_artifact):
    """Equal is not better, and the simpler thing wins ties."""
    assert gates.beats_baseline(make_artifact(test_median=106.8,
                                              baseline=106.8))


def test_a_regression_against_production_is_refused(make_artifact):
    """
    The model already running is the thing to beat.

    Beating the baseline is necessary and not sufficient: a model can
    clear a bar the incumbent cleared long ago and still be a downgrade.
    """
    incumbent = make_artifact(test_median=10.5)
    candidate = make_artifact(test_median=40.0)
    assert gates.no_regression(candidate, incumbent)


def test_a_small_regression_is_allowed(make_artifact):
    """
    A tolerance, not a wall.

    Accepting a slightly worse median for a gain elsewhere is sometimes
    right, and a gate that forbids it gets bypassed routinely - which
    costs the gate its meaning for the cases that matter.
    """
    incumbent = make_artifact(test_median=10.0)
    assert gates.no_regression(make_artifact(test_median=10.4),
                               incumbent) == []


def test_the_first_model_has_nothing_to_regress_against(good):
    """With no incumbent the gate passes rather than erroring."""
    assert gates.no_regression(good, None) == []


def test_reordered_features_are_refused_and_named_as_such(good):
    """
    ORDER IS THE CONTRACT.

    Same names in a different order runs perfectly and is wrong. The
    message has to say so, or whoever reads it will assume a typo.
    """
    swapped = [FEATURES[1], FEATURES[0]] + FEATURES[2:]
    reasons = gates.feature_contract(good, swapped)
    assert reasons and 'ORDER' in reasons[0]
    assert 'silently wrong' in reasons[0]


def test_different_features_are_refused(good):
    """A model fitted on columns the serving code does not compute."""
    assert gates.feature_contract(good, ['(u-cx)/w', '1/w'])


def test_a_model_without_provenance_is_refused(good):
    """
    A robot running an untraceable model cannot be explained.

    The question after an incident is which model, fitted on what, by
    which commit. An artifact that cannot answer makes the incident
    unresolvable rather than merely unpleasant.
    """
    good['code'] = {'commit': None}
    assert any('commit' in p for p in gates.provenance(good))

    good['code'] = {'commit': 'b' * 40}
    good['dataset']['code'] = {'commit': None}
    assert any('dataset' in p for p in gates.provenance(good))


def test_every_failure_is_reported_at_once(make_artifact):
    """
    Not the first one.

    A model failing three checks should be fixed once rather than three
    times, each after another full training run.
    """
    broken = make_artifact(test_median=200.0)
    broken['code'] = {'commit': None}
    failures = gates.check_all(broken, ['wrong', 'features'])
    assert set(failures) >= {'provenance', 'feature_contract',
                             'beats_baseline'}

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
Noticing that the robot is being asked a different question.

A model returns a number for whatever it is given, with the same
confidence for a scene it was fitted on and one it has never met. That
is the dangerous robot failure: not a crash, but a plausible answer.
"""

import pytest
from threevn_mlops.drift import DriftMonitor

STATS = {'names': ['a', 'b', 'c'],
         'mean': [0.0, 10.0, 100.0],
         'std': [1.0, 2.0, 5.0],
         'min': [-3.0, 4.0, 85.0],
         'max': [3.0, 16.0, 115.0]}


def test_a_typical_sample_is_not_drifting():
    """The training distribution's own centre must be unremarkable."""
    monitor = DriftMonitor(STATS)
    assert monitor.check([0.0, 10.0, 100.0])['drifting'] is False


def test_a_far_out_sample_is_flagged_with_the_feature_that_did_it():
    """
    Naming the feature is the difference between a warning and a lead.

    "Something is unusual" sends someone looking; "1/w is nine sigma out"
    says the target is a size the model never saw.
    """
    monitor = DriftMonitor(STATS)
    verdict = monitor.check([0.0, 10.0, 145.0])
    assert verdict['drifting'] is True
    assert verdict['worst_feature'] == 'c'
    assert verdict['max_z'] > 4.0


def test_values_outside_the_training_range_are_listed():
    """Beyond anything seen, which is stronger than merely unlikely."""
    verdict = DriftMonitor(STATS).check([-5.0, 10.0, 100.0])
    assert verdict['outside_training_range'] == ['a']


def test_a_feature_that_never_varied_cannot_judge(monkeypatch):
    """
    Zero spread means no opinion, not infinite suspicion.

    Dividing by it turns any difference at all into a drift alarm, and
    an alarm that fires constantly gets switched off - taking the real
    ones with it.
    """
    stats = dict(STATS, std=[0.0, 2.0, 5.0])
    verdict = DriftMonitor(stats).check([999.0, 10.0, 100.0])
    assert verdict['drifting'] is False


def test_the_wrong_number_of_features_is_an_error():
    """Better than scoring against whatever lines up."""
    with pytest.raises(ValueError, match='expected 3'):
        DriftMonitor(STATS).check([0.0, 10.0])


def test_the_summary_reports_rates_not_counts():
    """
    Rates mean something on their own.

    "Forty drifting frames" needs a denominator before it says anything;
    "forty percent" does not.
    """
    monitor = DriftMonitor(STATS)
    for _ in range(8):
        monitor.check([0.0, 10.0, 100.0])
    for _ in range(2):
        monitor.check([0.0, 10.0, 200.0])

    summary = monitor.summary()
    assert summary['frames'] == 10
    assert summary['fraction_drifting'] == pytest.approx(0.2)

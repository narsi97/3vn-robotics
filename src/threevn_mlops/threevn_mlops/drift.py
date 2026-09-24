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

A model has no way to say "I have not seen anything like this". It
returns a number for whatever it is given, with the same confidence for
a scene it was fitted on and one it has never met. On a robot that is
the dangerous failure: not a crash, but a plausible answer that moves an
arm to the wrong place.

The cheapest useful detector compares the features arriving now against
the features the model was fitted on, which Phase 14 records in the
artifact. It cannot tell you the answer is wrong - only that the
question has changed, which is when a wrong answer becomes likely.

Deliberately simple. A per-feature z-score against the training mean and
spread, and a count of features outside the training range entirely. No
distribution distance, no windowed statistical test: those need tuning
that this project has no data to justify, and an untuned detector that
fires constantly gets switched off.
"""

import numpy as np


class DriftMonitor:
    """Compare live features against the distribution a model was fitted on."""

    def __init__(self, feature_stats, z_threshold=4.0):
        self.names = list(feature_stats['names'])
        self.mean = np.array(feature_stats['mean'], dtype=np.float64)
        self.std = np.array(feature_stats['std'], dtype=np.float64)
        self.low = np.array(feature_stats['min'], dtype=np.float64)
        self.high = np.array(feature_stats['max'], dtype=np.float64)
        self.z_threshold = z_threshold

        # A feature that never varied in training has zero spread, and
        # dividing by it turns any difference at all into infinity. Treat
        # it as "cannot judge" rather than "always drifting".
        self.judgeable = self.std > 1e-12

        self.seen = 0
        self.out_of_range = 0
        self.beyond_threshold = 0

    def check(self, features):
        """
        Score one feature vector. Returns a dict describing it.

        `drifting` means this sample is unlike the training data, not
        that the model is broken. One odd frame is a reflection; a
        sustained rate of them is a different scene.
        """
        features = np.asarray(features, dtype=np.float64)
        if features.shape != self.mean.shape:
            raise ValueError(
                f'expected {len(self.mean)} features, got {features.shape}')

        z = np.zeros_like(features)
        z[self.judgeable] = np.abs(
            (features[self.judgeable] - self.mean[self.judgeable])
            / self.std[self.judgeable])

        outside = (features < self.low) | (features > self.high)
        worst = int(np.argmax(z)) if len(z) else 0

        self.seen += 1
        if outside.any():
            self.out_of_range += 1
        drifting = bool(z.max() > self.z_threshold) if len(z) else False
        if drifting:
            self.beyond_threshold += 1

        return {
            'drifting': drifting,
            'max_z': float(z.max()) if len(z) else 0.0,
            'worst_feature': self.names[worst] if self.names else None,
            'outside_training_range': [
                self.names[i] for i in np.nonzero(outside)[0]],
        }

    def summary(self):
        """Rates rather than counts, so the numbers mean something alone."""
        seen = max(self.seen, 1)
        return {
            'frames': self.seen,
            'fraction_drifting': self.beyond_threshold / seen,
            'fraction_outside_range': self.out_of_range / seen,
        }

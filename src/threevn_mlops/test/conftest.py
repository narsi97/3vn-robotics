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

"""Shared fixtures: a model artifact shaped like the real one."""

import copy

import pytest

FEATURES = ['(u-cx)/w', '(v-cy)/w', '1/w', 'h/w', 'sqrt(area)/w',
            '(h/w)/w', 'sqrt(area)/w^2']


def artifact(test_median=10.5, baseline=106.8):
    """Return a valid artifact with the given scores."""
    return {
        'schema_version': 1,
        'model': {'kind': 'ridge', 'alpha': 1e-06,
                  'weights': [[0.0] * 3] * len(FEATURES),
                  'intercept': [0.0, 0.0, 0.0]},
        'selected': 'ridge + shape',
        'features': list(FEATURES),
        'metrics': {
            'geometric': {'train': {'median_mm': baseline},
                          'test': {'median_mm': baseline, 'p95_mm': 155.5}},
            'ridge geometry': {'train': {'median_mm': 34.5},
                               'test': {'median_mm': 46.5, 'p95_mm': 122.2}},
            'ridge + shape': {'train': {'median_mm': 14.9},
                              'test': {'median_mm': test_median,
                                       'p95_mm': 74.8}},
        },
        'feature_stats': {'names': list(FEATURES),
                          'mean': [0.0] * len(FEATURES),
                          'std': [1.0] * len(FEATURES),
                          'min': [-1.0] * len(FEATURES),
                          'max': [1.0] * len(FEATURES)},
        'dataset': {'world': 'bench_with_target', 'robot': 'threevn_mm_v1',
                    'frames': 480, 'episodes': 40,
                    'recorded': '2026-09-23T23:12:48+00:00',
                    'code': {'commit': 'a' * 40, 'dirty': False},
                    'labels': {'source': 'simulator'}},
        'code': {'commit': 'b' * 40, 'dirty': False},
    }


@pytest.fixture
def good():
    """A model that should pass every gate."""
    return artifact()


@pytest.fixture
def make_artifact():
    """Build artifacts with chosen scores."""
    return lambda **kwargs: copy.deepcopy(artifact(**kwargs))

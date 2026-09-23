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

"""What the images are reduced to before anything is fitted."""

import numpy as np
import pytest
from threevn_ml import features as feat
from threevn_perception.detector import Blob, Intrinsics

K = Intrinsics(fx=532.57, fy=532.57, cx=320.0, cy=240.0)


def test_the_geometric_features_are_a_prefix_of_the_shape_features():
    """
    The ablation compares nested sets, so one must contain the other.

    Otherwise "adding shape helped" could be "changing the features
    helped", which is a different and weaker claim.
    """
    assert feat.SHAPE_FEATURES[:len(feat.GEOMETRIC_FEATURES)] == \
        feat.GEOMETRIC_FEATURES


def test_features_are_returned_in_the_order_requested():
    """
    Order is part of the contract with a fitted model.

    Feeding a model its columns in a different order than it was fitted
    on produces predictions rather than an error, which is why the names
    are stored in the artifact.
    """
    blob = Blob(u=400.0, v=300.0, width=60, height=70, area=3600)
    forward = feat.blob_features(blob, K, ('1/w', 'h/w'))
    backward = feat.blob_features(blob, K, ('h/w', '1/w'))
    assert forward[0] == pytest.approx(backward[1])
    assert forward[1] == pytest.approx(backward[0])


def test_the_geometric_features_match_their_definitions():
    """Arithmetic, checked against the formula they are named after."""
    blob = Blob(u=400.0, v=300.0, width=50, height=50, area=2500)
    values = dict(zip(feat.GEOMETRIC_FEATURES,
                      feat.blob_features(blob, K, feat.GEOMETRIC_FEATURES)))
    assert values['(u-cx)/w'] == pytest.approx((400.0 - 320.0) / 50)
    assert values['(v-cy)/w'] == pytest.approx((300.0 - 240.0) / 50)
    assert values['1/w'] == pytest.approx(1.0 / 50)


def test_the_aspect_ratio_carries_the_shape():
    """
    A square blob and a tall one differ, which is the whole point.

    The feature exists to let a model tell a cube seen face-on from one
    seen at an angle, because the two imply different effective widths.
    """
    square = Blob(u=320.0, v=240.0, width=50, height=50, area=2500)
    tall = Blob(u=320.0, v=240.0, width=50, height=70, area=3500)
    index = feat.SHAPE_FEATURES.index('h/w')
    assert feat.blob_features(square, K, feat.SHAPE_FEATURES)[index] == \
        pytest.approx(1.0)
    assert feat.blob_features(tall, K, feat.SHAPE_FEATURES)[index] > 1.0


def test_intrinsics_come_from_the_manifest():
    """The camera a dataset was recorded with, not one assumed."""
    manifest = {'camera': {'fx': 1.0, 'fy': 2.0, 'cx': 3.0, 'cy': 4.0}}
    assert feat.intrinsics_from_manifest(manifest) == \
        Intrinsics(fx=1.0, fy=2.0, cx=3.0, cy=4.0)


def test_an_undetectable_image_yields_no_features():
    """
    None, not zeros.

    A frame the detector cannot see is not a training example. Imputing
    a feature would invent one, and the model would fit the invention.
    """
    blank = np.zeros((48, 64, 3), dtype=np.uint8)
    assert feat.extract(blank, K) is None

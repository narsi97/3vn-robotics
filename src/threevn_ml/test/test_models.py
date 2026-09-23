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
The predictors, on data whose answer is known by construction.

No dataset, no images, no simulator: synthetic blobs placed by the exact
pinhole relation, so anything that fails here is the code and not the
world.
"""

import numpy as np
import pytest
from threevn_ml import features as feat
from threevn_ml import models
from threevn_perception.detector import Blob, Intrinsics

K = Intrinsics(fx=532.57, fy=532.57, cx=320.0, cy=240.0)
WIDTH = 0.05


def blob_for(x, y, z, width=WIDTH):
    """Return the blob a target at (x, y, z) would produce, exactly."""
    pixels = K.fx * width / z
    return Blob(u=K.cx + x * K.fx / z, v=K.cy + y * K.fy / z,
                width=pixels, height=pixels, area=pixels * pixels)


def features_for(points, names=feat.GEOMETRIC_FEATURES):
    """Return a feature matrix for a list of (x, y, z)."""
    return np.array([feat.blob_features(blob_for(*p), K, names)
                     for p in points])


POINTS = [(0.00, 0.00, 0.40), (0.10, -0.05, 0.35), (-0.08, 0.04, 0.55),
          (0.15, 0.02, 0.30), (-0.12, -0.06, 0.60), (0.05, 0.05, 0.45)]


# -- the closed form ----------------------------------------------------

def test_the_closed_form_inverts_the_projection_exactly():
    """
    Given a perfectly measured blob, the geometry is exact.

    This is the thing being compared against, so it has to be right. Any
    error it shows on real data is then a property of the measurement or
    of the constant W, not of the arithmetic.
    """
    truth = np.array(POINTS)
    predicted = models.Geometric(K, WIDTH).predict(features_for(POINTS))
    assert predicted == pytest.approx(truth, abs=1e-9)


def test_the_closed_form_is_wrong_by_the_ratio_of_the_width():
    """
    IT ASSUMES IT KNOWS HOW WIDE THE TARGET IS.

    Feeding it blobs from a target 30% wider - which is what an off-axis
    cube showing two faces looks like - biases every distance by exactly
    that ratio. On the real dataset the width implied by the images
    spans 55 to 78 mm against a declared 50, which is why the fitted
    models beat it.
    """
    wider = 1.30 * WIDTH
    observed = np.array([feat.blob_features(blob_for(*p, width=wider), K,
                                            feat.GEOMETRIC_FEATURES)
                         for p in POINTS])
    predicted = models.Geometric(K, WIDTH).predict(observed)
    assert predicted == pytest.approx(np.array(POINTS) / 1.30, rel=1e-9)


def test_fitting_the_baseline_changes_nothing():
    """It has nothing to learn; fit() exists so the loop can be uniform."""
    model = models.Geometric(K, WIDTH)
    before = model.predict(features_for(POINTS))
    model.fit(features_for(POINTS), np.array(POINTS))
    assert model.predict(features_for(POINTS)) == pytest.approx(before)


# -- the fitted model ---------------------------------------------------

def test_ridge_recovers_the_geometry_from_clean_data():
    """
    THE FEATURES WERE CHOSEN SO THAT THIS IS POSSIBLE.

    Every output is linear in them, so a linear model over exact
    measurements must reproduce the exact mapping. If this fails, the
    feature definitions and the projection disagree.
    """
    X = features_for(POINTS)
    model = models.Ridge(alpha=1e-12).fit(X, np.array(POINTS))
    assert model.predict(X) == pytest.approx(np.array(POINTS), abs=1e-6)


def test_ridge_absorbs_a_wrong_target_width():
    """
    The reason fitting beats the closed form.

    Blobs come from a target 30% wider than declared. The closed form
    cannot know; a fitted model sees consistent evidence and scales its
    weights accordingly.
    """
    wider = 1.30 * WIDTH
    X = np.array([feat.blob_features(blob_for(*p, width=wider), K,
                                     feat.GEOMETRIC_FEATURES)
                  for p in POINTS])
    truth = np.array(POINTS)

    closed = models.errors(models.Geometric(K, WIDTH).predict(X), truth)
    fitted = models.errors(
        models.Ridge(alpha=1e-12).fit(X, truth).predict(X), truth)
    assert fitted['median_mm'] < closed['median_mm'] / 10.0


def test_predicting_before_fitting_is_an_error():
    """Better than returning zeros, which look like a position."""
    with pytest.raises(RuntimeError, match='fit'):
        models.Ridge().predict(features_for(POINTS))


def test_the_penalty_does_not_drag_predictions_toward_the_camera():
    """
    The intercept is deliberately not penalised.

    Shrinking it would bias every prediction toward the origin, which for
    a position in the optical frame means toward the camera - a robot
    reaching consistently short.
    """
    X = features_for(POINTS)
    truth = np.array(POINTS)
    strong = models.Ridge(alpha=1.0).fit(X, truth)
    predicted = strong.predict(X)
    assert predicted[:, 2].mean() == pytest.approx(truth[:, 2].mean(), abs=1e-6)


def test_a_model_round_trips_through_its_artifact():
    """What is saved is what is loaded."""
    X = features_for(POINTS)
    model = models.Ridge(alpha=1e-9).fit(X, np.array(POINTS))
    restored = models.Ridge.from_dict(model.to_dict())
    assert restored.predict(X) == pytest.approx(model.predict(X))


# -- metrics ------------------------------------------------------------

def test_errors_reports_per_axis_and_distance():
    """
    Per-axis matters: the two failure modes are different.

    Phase 12 measured bearing accurate to a fraction of a millimetre and
    range biased by 30 mm. A single distance number hides exactly that.
    """
    truth = np.array([[0.0, 0.0, 0.40], [0.0, 0.0, 0.50]])
    predicted = truth + np.array([[0.0, 0.0, 0.02], [0.0, 0.0, 0.02]])
    summary = models.errors(predicted, truth)
    assert summary['median_mm'] == pytest.approx(20.0)
    assert summary['mae_x_mm'] == pytest.approx(0.0)
    assert summary['mae_z_mm'] == pytest.approx(20.0)
    assert summary['bias_z_mm'] == pytest.approx(20.0)


def test_bias_is_signed_and_absolute_error_is_not():
    """
    A model that is 20 mm long half the time and 20 mm short the other
    half has zero bias and 20 mm of error. Reporting only one of them
    describes a different model.
    """
    truth = np.array([[0.0, 0.0, 0.40], [0.0, 0.0, 0.40]])
    predicted = np.array([[0.0, 0.0, 0.42], [0.0, 0.0, 0.38]])
    summary = models.errors(predicted, truth)
    assert summary['bias_z_mm'] == pytest.approx(0.0)
    assert summary['mae_z_mm'] == pytest.approx(20.0)

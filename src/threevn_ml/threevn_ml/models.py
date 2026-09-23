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
Two predictors, and the rule that they are compared on equal terms.

`Geometric` is not a model. It is the closed-form deprojection from
Phase 12, wrapped in the same interface so it can be evaluated by the
same code on the same test set. A baseline that is described rather
than run is a baseline nobody checks, and "the model beat the geometry"
is not a claim until both numbers come out of one function.

`Ridge` is least squares with a penalty on the weights, solved
directly. There is no gradient descent, no epochs and no learning rate,
because for a linear model over three features the normal equations
have a closed-form solution and anything iterative would be ceremony
that can also be tuned into looking good.

WHY NOT A NEURAL NETWORK. The mapping is known to be linear in these
features - that is why they were chosen - so a network could at best
rediscover it while adding capacity to memorise 480 samples. If the
features were raw pixels the answer would be different, and so would
the amount of data needed.
"""

import json

import numpy as np


class Geometric:
    """
    The closed-form deprojection, as a predictor.

    Fitting does nothing: the only constant it could learn, the target's
    true width, is declared rather than estimated. That is exactly the
    difference this phase is measuring.
    """

    name = 'geometric'

    def __init__(self, intrinsics, target_width=0.05):
        self.intrinsics = intrinsics
        self.target_width = target_width

    def fit(self, features, targets):
        """Present so the evaluation loop can be uniform. Does nothing."""
        return self

    def predict(self, features):
        """
        Map features to (x, y, z).

        The features ARE the deprojection up to a scale factor, so this
        is one multiplication: x = ((u-cx)/w) * W, and likewise for y,
        while z = (1/w) * fx * W.
        """
        features = np.atleast_2d(features)
        scale = np.array([self.target_width,
                          self.target_width,
                          self.intrinsics.fx * self.target_width])
        return features * scale

    def to_dict(self):
        """Describe the predictor for an artifact."""
        return {'kind': 'geometric',
                'target_width': self.target_width,
                'fx': self.intrinsics.fx, 'fy': self.intrinsics.fy,
                'cx': self.intrinsics.cx, 'cy': self.intrinsics.cy}


class Ridge:
    """
    Least squares with a weight penalty, solved in closed form.

    The penalty matters less for fit quality here than for conditioning:
    the three features are strongly correlated, since all of them carry
    1/w, and an unpenalised solve on correlated columns produces large
    opposing weights that swing wildly between datasets. A small alpha
    keeps the coefficients near the physical values they correspond to,
    which is also what makes them readable afterwards.

    The intercept is NOT penalised. Shrinking it would bias every
    prediction toward zero, which for a position in metres means toward
    the camera.
    """

    name = 'ridge'

    def __init__(self, alpha=1e-6):
        self.alpha = alpha
        self.weights = None
        self.intercept = None

    def fit(self, features, targets):
        """Solve the normal equations."""
        features = np.atleast_2d(features)
        targets = np.atleast_2d(targets)

        # Centre both sides so the intercept falls out rather than being
        # fitted as a penalised weight.
        feature_mean = features.mean(axis=0)
        target_mean = targets.mean(axis=0)
        centred = features - feature_mean
        gram = centred.T @ centred + self.alpha * np.eye(centred.shape[1])
        self.weights = np.linalg.solve(gram, centred.T @ (targets - target_mean))
        self.intercept = target_mean - feature_mean @ self.weights
        return self

    def predict(self, features):
        """Apply the fitted linear map."""
        if self.weights is None:
            raise RuntimeError('fit() before predict()')
        return np.atleast_2d(features) @ self.weights + self.intercept

    def to_dict(self):
        """Describe the fitted model for an artifact."""
        return {'kind': 'ridge', 'alpha': self.alpha,
                'weights': self.weights.tolist(),
                'intercept': self.intercept.tolist()}

    @classmethod
    def from_dict(cls, doc):
        """Rebuild a model from its artifact."""
        model = cls(alpha=doc['alpha'])
        model.weights = np.array(doc['weights'], dtype=np.float64)
        model.intercept = np.array(doc['intercept'], dtype=np.float64)
        return model


def errors(predicted, actual):
    """
    Return the error summary both predictors are judged by.

    Median and p95 rather than mean: with a handful of frames where the
    segmentation clipped, a mean says more about those than about
    typical behaviour, and the typical behaviour is what a robot
    reaching for something experiences.

    Per-axis absolute error is reported alongside the 3D distance
    because the two failure modes are different. Phase 12 measured
    bearing accurate to a fraction of a millimetre and range biased by
    30 mm; a single distance number hides exactly that.
    """
    predicted = np.atleast_2d(predicted)
    actual = np.atleast_2d(actual)
    delta = predicted - actual
    distance = np.linalg.norm(delta, axis=1)
    return {
        'n': int(len(distance)),
        'median_mm': float(np.median(distance) * 1000.0),
        'p95_mm': float(np.percentile(distance, 95) * 1000.0),
        'max_mm': float(distance.max() * 1000.0),
        'mae_x_mm': float(np.abs(delta[:, 0]).mean() * 1000.0),
        'mae_y_mm': float(np.abs(delta[:, 1]).mean() * 1000.0),
        'mae_z_mm': float(np.abs(delta[:, 2]).mean() * 1000.0),
        'bias_z_mm': float(delta[:, 2].mean() * 1000.0),
    }


def save(path, document):
    """Write a model artifact."""
    with open(path, 'w') as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write('\n')


def load(path):
    """Read a model artifact."""
    with open(path) as handle:
        return json.load(handle)

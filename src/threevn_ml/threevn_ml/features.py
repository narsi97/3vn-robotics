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
Turning an image into a handful of numbers a model can fit.

WHY NOT RAW PIXELS. A 640x480 RGB frame is 921,600 numbers and this
dataset has 480 of them. Fitting anything to that needs either a
convolutional architecture and far more data, or a smaller
representation. The smaller representation is available for free: the
Phase 12 detector already reduces the frame to a blob, and the blob is
what the geometry uses.

WHY THESE FEATURES IN PARTICULAR. The deprojection is

    z = fx * W / w
    x = (u - cx) * z / fx  =  (u - cx) * W / w
    y = (v - cy) * z / fy  =  (v - cy) * W / w

for a target of true width W seen `w` pixels across. Every output is
LINEAR in the three quantities

    (u - cx) / w,   (v - cy) / w,   1 / w

so a linear model over them can express the exact geometry - and, unlike
the closed form, it can also fit W. That matters, because the closed
form's error is dominated by segmentation taking in about three pixels
of edge bleed per side, which behaves exactly like the target being
slightly wider than it is.

That is the honest case for fitting a model here: not that the geometry
is unknown, but that one of its constants is wrong in a way the data can
correct.
"""

import numpy as np

from threevn_perception.detector import (
    Intrinsics,
    colour_mask,
    largest_blob,
)

#: The closed form, exactly. A linear model over these can reproduce the
#: geometry and fit its one constant.
GEOMETRIC_FEATURES = ('(u-cx)/w', '(v-cy)/w', '1/w')

#: The same, plus shape.
#:
#: WHY SHAPE IS NEEDED, measured rather than assumed. The closed form
#: divides by the target's true width W, taken as the 50 mm cube edge.
#: That is right only head-on. Seen at an angle a cube shows two faces
#: and its silhouette widens, up to the 70.7 mm diagonal. On this
#: dataset the width implied by w * z / fx has a median of 64.9 mm and
#: spans 55 to 78 mm - bracketing exactly that range, plus a little
#: edge bleed.
#:
#: So W is not a constant; it varies with viewing angle. No model over
#: GEOMETRIC_FEATURES can represent that, because nothing in them says
#: which way the cube is facing. The blob's aspect ratio does: a
#: square-on cube is as tall as it is wide, and a rotated one is not.
#:
#: The interaction terms exist because the correction is multiplicative.
#: z = fx * W(angle) / w, so the model needs (1/w) scaled by something
#: carrying the angle, not added to it.
SHAPE_FEATURES = GEOMETRIC_FEATURES + (
    'h/w', 'sqrt(area)/w', '(h/w)/w', 'sqrt(area)/w^2')

#: The set used unless a caller says otherwise. Stored in the model
#: artifact so a model can never be fed its features in a different
#: order than it was fitted on, which produces predictions rather than
#: an error.
FEATURE_NAMES = SHAPE_FEATURES


def blob_features(blob, intrinsics, names=FEATURE_NAMES):
    """Return the named features for one detected blob."""
    width = float(blob.width)
    height = float(blob.height)
    root_area = float(blob.area) ** 0.5
    available = {
        '(u-cx)/w': (blob.u - intrinsics.cx) / width,
        '(v-cy)/w': (blob.v - intrinsics.cy) / width,
        '1/w': 1.0 / width,
        'h/w': height / width,
        'sqrt(area)/w': root_area / width,
        '(h/w)/w': height / (width * width),
        'sqrt(area)/w^2': root_area / (width * width),
    }
    return np.array([available[name] for name in names], dtype=np.float64)


def extract(rgb, intrinsics, names=FEATURE_NAMES, **mask_kwargs):
    """Return features for one image, or None if nothing was detected."""
    blob = largest_blob(colour_mask(rgb, **mask_kwargs))
    if blob is None:
        return None
    return blob_features(blob, intrinsics, names)


def intrinsics_from_manifest(manifest):
    """Read the camera the dataset was recorded with."""
    camera = manifest['camera']
    return Intrinsics(fx=camera['fx'], fy=camera['fy'],
                      cx=camera['cx'], cy=camera['cy'])

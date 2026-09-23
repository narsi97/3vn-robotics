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
The detector, on images this file draws itself.

Every case here is a picture with a known answer, so a failure says what
is wrong rather than that a simulator produced something unexpected.
These run in milliseconds and need no ROS, no camera and no Gazebo.
"""

import numpy as np
import pytest
from threevn_perception.detector import (
    Blob,
    as_rgb,
    colour_mask,
    detect,
    intrinsics_from_fov,
    largest_blob,
    position_in_optical_frame,
    range_from_apparent_size,
)

WIDTH, HEIGHT = 640, 480
HFOV = 62.0

#: The robot's own livery, from the description's Gazebo materials.
ROBOT_ORANGE = (230, 125, 33)
ROBOT_DARK = (31, 33, 41)
ROBOT_GREY = (61, 69, 82)
ROBOT_LIGHT = (199, 204, 212)
GROUND_GREY = (89, 94, 102)

TARGET_GREEN = (20, 217, 51)


def scene(background=GROUND_GREY):
    """Return a blank image of the simulated ground colour."""
    image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    image[:, :] = background
    return image


def draw(image, u, v, size, colour=TARGET_GREEN):
    """Paint a square centred on (u, v)."""
    half = size // 2
    image[v - half:v + half, u - half:u + half] = colour
    return image


@pytest.fixture
def intrinsics():
    """Provide intrinsics matching the simulated camera."""
    return intrinsics_from_fov(WIDTH, HEIGHT, HFOV)


# -- intrinsics ---------------------------------------------------------

def test_intrinsics_match_the_pinhole_relation(intrinsics):
    """fx = (width/2) / tan(hfov/2), and the centre is the centre."""
    assert intrinsics.fx == pytest.approx(532.57, abs=0.05)
    assert intrinsics.fy == pytest.approx(intrinsics.fx)
    assert (intrinsics.cx, intrinsics.cy) == (320.0, 240.0)


def test_a_wider_lens_has_a_shorter_focal_length(intrinsics):
    """The direction of the relation, so a reciprocal slip cannot hide."""
    wide = intrinsics_from_fov(WIDTH, HEIGHT, 90.0)
    assert wide.fx < intrinsics.fx


# -- encoding -----------------------------------------------------------

def test_bgr_is_reordered_not_reinterpreted():
    """
    bgr8 and rgb8 differ, and treating one as the other finds nothing.

    A colour detector fed swapped channels does not crash or look broken.
    It quietly stops seeing green.
    """
    image = draw(scene(), 320, 240, 40)
    swapped = image[:, :, ::-1].tobytes()
    restored = as_rgb(swapped, HEIGHT, WIDTH, 'bgr8')
    assert np.array_equal(restored, image)


def test_an_unknown_encoding_is_refused():
    """Better a loud error than a silent misread."""
    with pytest.raises(ValueError, match='unsupported encoding'):
        as_rgb(b'\x00' * (HEIGHT * WIDTH * 3), HEIGHT, WIDTH, 'mono8')


# -- masking ------------------------------------------------------------

def test_the_target_is_found_against_the_ground():
    """The basic case: one green square on grey."""
    mask = colour_mask(draw(scene(), 320, 240, 40))
    assert mask.sum() == pytest.approx(40 * 40, rel=0.05)


@pytest.mark.parametrize('colour,name', [
    (ROBOT_ORANGE, 'orange'), (ROBOT_DARK, 'dark'),
    (ROBOT_GREY, 'grey'), (ROBOT_LIGHT, 'light grey'),
    (GROUND_GREY, 'ground'),
])
def test_the_robot_s_own_colours_are_not_detected(colour, name):
    """
    THE ROBOT MUST NOT DETECT ITSELF.

    The arm swings through frame constantly. If any part of its own
    livery passes the threshold, the detector reports a target that is
    its own elbow - intermittently, which is the worst way for a test to
    fail.

    The target is green precisely because the robot wears orange, which
    in hue terms sits close to red and would collide with the obvious
    first choice of a red target.
    """
    mask = colour_mask(scene(background=colour))
    assert not mask.any(), f'{name} {colour} was detected as the target'


def test_a_white_highlight_is_not_detected():
    """
    Specular highlights are bright in every channel, not green.

    This is why the mask tests DOMINANCE rather than brightness. A
    threshold on the green channel alone calls every white pixel a
    target, and a rendered scene is full of them.
    """
    assert not colour_mask(scene(background=(250, 250, 250))).any()


# -- blobs --------------------------------------------------------------

def test_the_blob_centroid_is_where_the_square_was():
    """Centroid, size and area all recovered from a known square."""
    blob = largest_blob(colour_mask(draw(scene(), 200, 150, 40)))
    assert blob.u == pytest.approx(200, abs=1.0)
    assert blob.v == pytest.approx(150, abs=1.0)
    assert blob.width == pytest.approx(40, abs=1)
    assert blob.area == pytest.approx(1600, rel=0.05)


def test_two_targets_do_not_average_into_a_phantom():
    """
    WITH TWO OBJECTS, THE ANSWER MUST BE ONE OF THEM.

    Taking the centroid of every masked pixel is the obvious shortcut. It
    puts the detection midway between the two squares, on empty ground,
    and the result looks like an ordinary detection in every way except
    being of nothing at all.
    """
    image = draw(scene(), 150, 240, 40)
    draw(image, 490, 240, 60)
    blob = largest_blob(colour_mask(image))
    assert blob.u == pytest.approx(490, abs=2.0), (
        'expected the LARGER blob at u=490, not a phantom between the two'
    )


def test_noise_below_the_area_floor_is_ignored():
    """A handful of stray pixels is not an object."""
    image = scene()
    image[10, 10] = TARGET_GREEN
    image[11, 11] = TARGET_GREEN
    assert largest_blob(colour_mask(image)) is None


def test_an_empty_scene_detects_nothing():
    """No target, no answer. Not an exception, and not a zero."""
    assert largest_blob(colour_mask(scene())) is None


# -- geometry -----------------------------------------------------------

def test_range_follows_the_pinhole_relation(intrinsics):
    """A 50 mm cube 74 px wide sits about 360 mm away."""
    distance = range_from_apparent_size(74, 0.05, intrinsics.fx)
    assert distance == pytest.approx(0.36, abs=0.005)


def test_a_closer_object_looks_bigger(intrinsics):
    """Direction of the relation, so an inverted formula cannot pass."""
    near = range_from_apparent_size(148, 0.05, intrinsics.fx)
    far = range_from_apparent_size(37, 0.05, intrinsics.fx)
    assert near < far
    assert far == pytest.approx(4 * near, rel=1e-6)


def test_a_centred_object_is_straight_ahead(intrinsics):
    """On the optical axis, x and y are zero and all the range is z."""
    blob = Blob(u=320.0, v=240.0, width=74, height=74, area=5476)
    x, y, z = position_in_optical_frame(blob, intrinsics, 0.05)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(0.36, abs=0.005)


def test_right_in_the_image_is_positive_x(intrinsics):
    """REP-103 optical: +x is to the right, matching increasing u."""
    blob = Blob(u=420.0, v=240.0, width=74, height=74, area=5476)
    x, _, _ = position_in_optical_frame(blob, intrinsics, 0.05)
    assert x > 0


def test_lower_in_the_image_is_positive_y(intrinsics):
    """
    +y is DOWN in the optical frame, and v also increases downward.

    So the two share a sign and there is no flip. The code that looks
    like it forgot one is right, and adding it puts every detection on
    the wrong side of the image - which is why this is asserted rather
    than left to a reader's judgement.
    """
    blob = Blob(u=320.0, v=340.0, width=74, height=74, area=5476)
    _, y, _ = position_in_optical_frame(blob, intrinsics, 0.05)
    assert y > 0


def test_the_full_pipeline_recovers_a_placed_target(intrinsics):
    """One call, one known answer."""
    found = detect(draw(scene(), 420, 180, 74), intrinsics, 0.05)
    assert found is not None
    blob, (x, y, z) = found
    assert blob.u == pytest.approx(420, abs=1.0)
    assert z == pytest.approx(0.36, abs=0.01)
    assert x > 0, 'right of centre must be +x'
    assert y < 0, 'above centre must be -y'


def test_the_pipeline_returns_none_rather_than_guessing(intrinsics):
    """An empty scene produces no detection at all."""
    assert detect(scene(), intrinsics, 0.05) is None


# -- what the threshold does, and does not, move ------------------------
#
# Measured against the simulator: sweeping the dominance threshold across
# its usable band moved the horizontal centroid by 0.00 px and left the
# bounding box unchanged, while pushing past that band collapsed both.
# These reproduce that on a drawn image with a soft edge, because it is
# the property the range accuracy depends on.

def soft_edged_target(u, v, size, bleed=3):
    """
    Draw a square whose edge fades into the background.

    A rendered object does not have a hard edge: the boundary pixels are
    a blend of object and background, which is exactly what makes the
    segmented width depend on where the threshold cuts.
    """
    image = scene().astype(np.float64)
    half = size // 2
    for offset in range(bleed + 1):
        weight = 1.0 - offset / (bleed + 1.0)
        lo, hi = half + offset, half + offset
        block = image[v - lo:v + hi, u - lo:u + hi]
        blended = (np.array(TARGET_GREEN) * weight
                   + np.array(GROUND_GREY) * (1.0 - weight))
        block[:, :] = np.maximum(block, blended)
    image[v - half:v + half, u - half:u + half] = TARGET_GREEN
    return image.astype(np.uint8)


@pytest.mark.parametrize('dominance', [20, 30, 45, 60])
def test_the_centroid_does_not_move_with_the_threshold(dominance):
    """
    BEARING IS THE TRUSTWORTHY OUTPUT.

    A soft edge fades symmetrically, so tightening the threshold removes
    the same ring of pixels all the way round and the centre of mass
    stays where it was. This is why the lateral position is accurate to
    well under a millimetre while the range is not.
    """
    image = soft_edged_target(300, 200, 40)
    blob = largest_blob(colour_mask(image, dominance=dominance))
    assert blob is not None
    assert blob.u == pytest.approx(300, abs=1.0)
    assert blob.v == pytest.approx(200, abs=1.0)


def test_the_apparent_width_does_move_with_the_threshold():
    """
    RANGE IS THE UNTRUSTWORTHY ONE, AND THIS IS WHY.

    The width is measured at the edge, which is where the ambiguity
    lives. A looser threshold eats into the bleed and reports a wider
    object, and a wider object is read as a nearer one. Nothing in the
    pinhole arithmetic is wrong; the input to it is.
    """
    image = soft_edged_target(300, 200, 40, bleed=4)
    loose = largest_blob(colour_mask(image, dominance=10))
    tight = largest_blob(colour_mask(image, dominance=70))
    assert loose.width > tight.width, (
        'a looser threshold must include more of the soft edge'
    )


def test_a_wider_blob_is_reported_as_nearer(intrinsics):
    """
    The consequence, stated as arithmetic.

    Over-segmenting by a few pixels per side biases the range SHORT, and
    the effect grows as the target shrinks with distance because the
    bleed stays a few pixels wide while the target does not.
    """
    true_width_px = 68
    over_segmented = 74  # what the simulator actually yields at 0.39 m

    true_range = range_from_apparent_size(true_width_px, 0.05, intrinsics.fx)
    biased = range_from_apparent_size(over_segmented, 0.05, intrinsics.fx)

    assert biased < true_range
    assert (true_range - biased) == pytest.approx(0.030, abs=0.004), (
        'the measured 30 mm shortfall should follow from 6 px of extra width'
    )

    # Same 6 px of bleed on a target half the size, twice as far away.
    far_true = range_from_apparent_size(34, 0.05, intrinsics.fx)
    far_biased = range_from_apparent_size(40, 0.05, intrinsics.fx)
    assert (far_true - far_biased) > (true_range - biased), (
        'the same edge bleed must cost more range error further away'
    )

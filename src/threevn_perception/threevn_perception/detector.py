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
Finding a coloured object, as arithmetic.

Nothing here imports rclpy, touches a topic or needs a simulator. A
detector that can only be run by starting Gazebo and waiting for a frame
is a detector nobody tests, and perception is precisely where a swapped
axis or a wrong sign produces output that looks entirely reasonable and
is wrong by ninety degrees.

Everything below operates on a plain (height, width, 3) uint8 array and
returns plain numbers.
"""

import collections
import math

import numpy as np

#: A blob found in the image. Pixel coordinates, image convention:
#: u rightward from the left edge, v DOWNWARD from the top.
Blob = collections.namedtuple('Blob', 'u v width height area')

#: Pinhole intrinsics, as CameraInfo carries them.
Intrinsics = collections.namedtuple('Intrinsics', 'fx fy cx cy')


def intrinsics_from_camera_info(msg):
    """
    Read intrinsics from a CameraInfo message.

    Taken from the message rather than recomputed from the robot profile
    on purpose: the profile says what the camera SHOULD be, and this says
    what it is. A test cross-checks the two, which is how a resolution
    change that never reached the simulator gets noticed.
    """
    return Intrinsics(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])


def intrinsics_from_fov(width, height, hfov_deg):
    """
    Derive pinhole intrinsics from an image size and horizontal field of view.

    The same arithmetic a simulator does when it turns a <horizontal_fov>
    into a projection matrix:

        fx = (width / 2) / tan(hfov / 2)

    fy equals fx for square pixels, which is what a rendered camera has
    and what a real one approximately has.
    """
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return Intrinsics(fx=fx, fy=fx, cx=width / 2.0, cy=height / 2.0)


def as_rgb(buffer, height, width, encoding):
    """
    Return an (h, w, 3) RGB array from a raw image buffer.

    Only the two encodings this project actually produces are accepted,
    and anything else raises rather than being quietly reinterpreted.

    cv_bridge would do this and much more. It is not used because the
    "much more" - compressed transport, bayer, 16-bit depth - is not
    needed here, and keeping this a pure array function is what lets
    every test below run with no ROS at all. If compressed transport
    arrives, cv_bridge is the right answer and this is the place to put
    it.
    """
    array = np.frombuffer(buffer, dtype=np.uint8).reshape(height, width, 3)
    if encoding == 'rgb8':
        return array
    if encoding == 'bgr8':
        return array[:, :, ::-1]
    raise ValueError(
        f'unsupported encoding {encoding!r}; this detector handles rgb8 and '
        f'bgr8. Silently treating one as the other swaps red and blue, '
        f'which for a colour detector means finding nothing at all.')


def colour_mask(rgb, channel=1, minimum=60, dominance=30):
    """
    Return a boolean mask of pixels where one channel clearly dominates.

    Deliberately not an HSV threshold. Hue is the textbook answer and it
    is unstable exactly where this is used: on a rendered scene with
    specular highlights and shadow, hue swings wildly in near-white and
    near-black pixels, so an HSV band needs saturation and value guards
    that amount to this check written less directly.

    `dominance` is what keeps grey out. Ground and robot are desaturated,
    so their channels sit close together; a saturated target separates
    them by far more than this margin.
    """
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f'expected an (h, w, 3) image, got {rgb.shape}')

    image = rgb.astype(np.int16)
    target = image[:, :, channel]
    others = [image[:, :, i] for i in range(3) if i != channel]
    mask = target > minimum
    for other in others:
        mask &= (target - other) > dominance
    return mask


def largest_blob(mask, min_area=40):
    """
    Return the largest connected region of `mask`, or None.

    Connectivity matters. Taking the centroid of every masked pixel is
    the obvious shortcut and it fails the moment two objects of the same
    colour are visible: the answer lands between them, on nothing, and
    looks like a perfectly ordinary detection.

    Labelling is done here rather than with scipy or OpenCV to keep this
    module free of a dependency that exists only for one call. It is an
    iterative flood fill over a boolean array, which is fast enough for
    one 640x480 frame at 15 Hz and easy to read.
    """
    if not mask.any():
        return None

    height, width = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    best = None

    for start_y, start_x in zip(*np.nonzero(mask)):
        if seen[start_y, start_x]:
            continue
        # Iterative, not recursive: a 640x480 blob would blow the stack.
        stack = [(start_y, start_x)]
        seen[start_y, start_x] = True
        pixels = []
        while stack:
            y, x = stack.pop()
            pixels.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if (0 <= ny < height and 0 <= nx < width
                        and mask[ny, nx] and not seen[ny, nx]):
                    seen[ny, nx] = True
                    stack.append((ny, nx))

        if best is None or len(pixels) > len(best):
            best = pixels

    if best is None or len(best) < min_area:
        return None

    ys = np.array([p[0] for p in best], dtype=np.float64)
    xs = np.array([p[1] for p in best], dtype=np.float64)
    return Blob(
        u=float(xs.mean()),
        v=float(ys.mean()),
        width=int(xs.max() - xs.min()) + 1,
        height=int(ys.max() - ys.min()) + 1,
        area=len(best),
    )


def range_from_apparent_size(pixel_width, real_width, fx):
    """
    Return the distance along the optical axis, from how large the object looks.

    The pinhole relation, rearranged:

        pixel_width = fx * real_width / distance

    This needs the object's true size to be known in advance, which is a
    real limitation and the honest one to state: it works for a cube of
    declared dimensions and not for an arbitrary object. A depth camera
    or a second viewpoint removes the assumption; neither exists in this
    phase.

    Accuracy degrades with distance, because `pixel_width` is quantised.
    At 74 px the next pixel is worth about 5 mm; at 20 px it is worth
    about 18 mm. Reporting a distance to the millimetre would be
    arithmetic pretending to be measurement.
    """
    if pixel_width <= 0:
        raise ValueError('pixel_width must be positive')
    return fx * real_width / pixel_width


def position_in_optical_frame(blob, intrinsics, real_width):
    """
    Return the object's (x, y, z) in the camera's OPTICAL frame, in metres.

    REP-103 optical convention: +z forward along the view direction, +x
    right, +y DOWN. That last one is the trap. Image v also increases
    downward, so y and v share a sign and no flip is needed -- which
    means the code that looks like it is missing a flip is correct, and
    "fixing" it puts every detection on the wrong side of the image.
    """
    distance = range_from_apparent_size(blob.width, real_width, intrinsics.fx)
    x = (blob.u - intrinsics.cx) * distance / intrinsics.fx
    y = (blob.v - intrinsics.cy) * distance / intrinsics.fy
    return x, y, distance


def detect(rgb, intrinsics, real_width, **mask_kwargs):
    """
    Find the target and return (Blob, position) or None.

    The whole pipeline, as one call over one array.
    """
    blob = largest_blob(colour_mask(rgb, **mask_kwargs))
    if blob is None:
        return None
    return blob, position_in_optical_frame(blob, intrinsics, real_width)

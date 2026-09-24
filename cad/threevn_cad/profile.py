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
Reading the robot profile, in the units CAD works in.

THE URDF IS IN METRES AND EVERY PRINTER, DATASHEET AND M3 SCREW IS IN
MILLIMETRES. That conversion has to happen exactly once, in one place,
or it happens three times with one of them wrong - and a part 1000x too
large still exports, still opens, and still looks fine until someone
checks the scale bar.

So: this module is the only place that multiplies by 1000, the CAD
functions take millimetres throughout, and a test asserts a known link
length survives the trip.
"""

import pathlib

import yaml

#: The profile is the single source of truth for the robot's dimensions;
#: the CAD reads the same file the URDF does.
DEFAULT_PROFILE = (pathlib.Path(__file__).resolve().parents[2]
                   / 'src' / 'threevn_robot_description' / 'config'
                   / 'threevn_arm_v1.yaml')

MM_PER_M = 1000.0


def load(path=None):
    """Read a robot profile."""
    path = pathlib.Path(path) if path else DEFAULT_PROFILE
    if not path.is_file():
        raise FileNotFoundError(
            f'{path} does not exist. The CAD reads the SAME profile the '
            f'URDF does, so that the printed part and the simulated model '
            f'cannot disagree.')
    return yaml.safe_load(path.read_text())


def mm(metres):
    """Convert metres to millimetres. The only conversion in the CAD."""
    return float(metres) * MM_PER_M


def link_mm(cfg, name):
    """
    Return a link's geometry in millimetres, whatever its shape.

    Boxes carry x/y/z; cylinders carry radius/length. Returning a dict
    rather than a tuple keeps the caller honest about which it asked
    for: a cylinder unpacked as a box silently becomes a very strange
    bracket.
    """
    geometry = cfg['links'][name]['geometry']
    kind = geometry['type']
    if kind == 'box':
        return {'type': 'box',
                'x': mm(geometry['x']), 'y': mm(geometry['y']),
                'z': mm(geometry['z'])}
    if kind == 'cylinder':
        return {'type': 'cylinder',
                'radius': mm(geometry['radius']),
                'length': mm(geometry['length'])}
    raise ValueError(f'{name}: unsupported geometry type {kind!r}')


def servo(cfg, name):
    """
    Return a servo's physical envelope, already in millimetres.

    The datasheet block is in millimetres and stays that way: putting a
    unit conversion between a datasheet and the pocket that has to hold
    the part is how a servo ends up not fitting for a reason nobody can
    see in the drawing.
    """
    spec = cfg['servos'][name]
    if 'body' not in spec:
        raise KeyError(
            f'servo {name!r} has no `body` block. The CAD needs a physical '
            f'envelope, not just torque and mass.')
    return dict(spec['body'])


def fabrication(cfg):
    """Return the printer and fastener constants, in millimetres."""
    if 'fabrication' not in cfg:
        raise KeyError(
            'the profile has no `fabrication` block. Wall thickness and '
            'hole clearance describe the PRINTER, not the robot, and the '
            'CAD cannot guess them.')
    return dict(cfg['fabrication'])


def joint_servo(cfg, joint):
    """Return the servo name a joint is driven by."""
    spec = cfg['joints'][joint]
    if 'servo' not in spec:
        raise KeyError(f'joint {joint!r} declares no servo')
    return spec['servo']

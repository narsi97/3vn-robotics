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
Survey where this arm can actually put its gripper.

    ros2 run threevn_control workspace

Reach is not the answer to that question. Reach is a sphere of radius
a1+a2+a3 and the arm cannot use most of it, because a grasp needs the
gripper pointing a particular way and the joint limits do not allow
every direction everywhere. Asking "can it get there" without saying
"pointing which way" produces a map of places the arm can touch but
cannot pick from.

So this surveys the workspace at a GIVEN tool pitch, and separately at
any pitch at all, and prints both. The gap between them is the part of
the reachable volume that is useless for top-down picking.

This matters before building anything that sorts objects: it is what
tells you where the objects and the bins have to be. Guessing, and
discovering later that half the table cannot be reached with the gripper
pointing down, is the expensive order to find out.
"""

import argparse
import math
import pathlib
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
from threevn_control.kinematics import (
    arm_geometry,
    inverse_kinematics,
    Unreachable,
)

#: Straight down: the gripper approaches a table from above.
TOP_DOWN = math.pi

#: Pitches tried when asking "any orientation at all". Coarse on
#: purpose -- this answers "is there one", not "which is best".
PITCH_SAMPLES = tuple(math.radians(d) for d in range(60, 221, 10))


def expand(profile=None):
    """Return the expanded URDF for a profile, defaulting to the arm."""
    import xacro
    share = pathlib.Path(
        get_package_share_directory('threevn_robot_description'))
    profile = profile or share / 'config' / 'threevn_arm_v1.yaml'
    doc = xacro.process_file(
        str(share / 'urdf' / 'threevn_arm.urdf.xacro'),
        mappings={'params_file': str(profile), 'target': 'mock'})
    return doc.toprettyxml(indent='  ')


def feasible_pitches(urdf, position, geometry=None, pitches=PITCH_SAMPLES):
    """Return the sampled pitches that can place the tool at `position`."""
    found = []
    for pitch in pitches:
        try:
            inverse_kinematics(urdf, position, pitch=pitch, geometry=geometry)
            found.append(pitch)
        except Unreachable:
            continue
    return found


def survey(urdf, pitch=TOP_DOWN, geometry=None, step=0.01,
           radius=0.30, z_range=(-0.05, 0.32)):
    """
    Return a grid of (radius, height) -> reachable, in the arm's own plane.

    Surveyed in the vertical plane the arm swings through rather than in
    x/y/z: the first joint turns the whole arm, so the workspace is a
    surface of revolution and a 3D sweep would compute the same answer
    once per yaw angle.
    """
    geom = geometry or arm_geometry(urdf)
    radii = np.arange(0.0, radius + step, step)
    heights = np.arange(z_range[0], z_range[1] + step, step)

    at_pitch = np.zeros((len(heights), len(radii)), dtype=bool)
    any_pitch = np.zeros_like(at_pitch)

    for row, z in enumerate(heights):
        for col, r in enumerate(radii):
            point = (float(r), 0.0, float(z))
            if r < 1e-6:
                continue
            try:
                inverse_kinematics(urdf, point, pitch=pitch, geometry=geom)
                at_pitch[row, col] = True
            except Unreachable:
                pass
            any_pitch[row, col] = bool(
                at_pitch[row, col]
                or feasible_pitches(urdf, point, geometry=geom))

    return {
        'radii': radii,
        'heights': heights,
        'at_pitch': at_pitch,
        'any_pitch': any_pitch,
        'pitch': pitch,
        'geometry': geom,
    }


def bounds(grid, key='at_pitch'):
    """Return the (min, max) radius and height where `key` is reachable."""
    mask = grid[key]
    if not mask.any():
        return None
    rows, cols = np.nonzero(mask)
    return {
        'radius': (float(grid['radii'][cols.min()]),
                   float(grid['radii'][cols.max()])),
        'height': (float(grid['heights'][rows.min()]),
                   float(grid['heights'][rows.max()])),
        'cells': int(mask.sum()),
    }


def render(grid):
    """Return an ASCII map: '#' reachable at pitch, '+' only at some other."""
    lines = []
    for row in range(len(grid['heights']) - 1, -1, -1):
        z = grid['heights'][row]
        cells = ''.join(
            '#' if grid['at_pitch'][row, col]
            else ('+' if grid['any_pitch'][row, col] else '.')
            for col in range(len(grid['radii'])))
        lines.append(f'  z={z * 1000:+6.0f}mm |{cells}|')
    ruler = ''.join('|' if abs(r * 100 - round(r * 100)) < 1e-9
                    and round(r * 1000) % 50 == 0 else ' '
                    for r in grid['radii'])
    lines.append(f'  {"":13s}|{ruler}|')
    lines.append(f'  {"":14s}0{" " * (len(grid["radii"]) - 6)}'
                 f'{grid["radii"][-1] * 1000:.0f}mm radius')
    return '\n'.join(lines)


def main(argv=None):
    """Print the reachable workspace for a profile."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default=None,
                        help='path to a robot profile YAML')
    parser.add_argument('--pitch-deg', type=float, default=180.0,
                        help='tool pitch: 180 is straight down')
    parser.add_argument('--step', type=float, default=0.01,
                        help='grid spacing in metres')
    args = parser.parse_args(argv)

    urdf = expand(pathlib.Path(args.profile) if args.profile else None)
    geom = arm_geometry(urdf)
    pitch = math.radians(args.pitch_deg)

    reach = geom['a1'] + geom['a2'] + geom['a3']
    print()
    print(f'  arm        a1={geom["a1"] * 1000:.0f}  a2={geom["a2"] * 1000:.0f}'
          f'  a3={geom["a3"] * 1000:.0f} mm,'
          f' shoulder {geom["shoulder_z"] * 1000:.0f} mm up')
    print(f'  reach      {reach * 1000:.0f} mm, ignoring orientation')
    print()

    grid = survey(urdf, pitch=pitch, geometry=geom, step=args.step)
    print(f'  # = gripper can point at {args.pitch_deg:.0f} deg '
          f'(180 = straight down)')
    print('  + = reachable, but only pointing some other way')
    print()
    print(render(grid))
    print()

    at = bounds(grid, 'at_pitch')
    any_ = bounds(grid, 'any_pitch')
    if at is None:
        print(f'  NOTHING is reachable at {args.pitch_deg:.0f} deg.')
    else:
        print(f'  at {args.pitch_deg:.0f} deg: radius '
              f'{at["radius"][0] * 1000:.0f}-{at["radius"][1] * 1000:.0f} mm, '
              f'height {at["height"][0] * 1000:+.0f} to '
              f'{at["height"][1] * 1000:+.0f} mm')
    if any_:
        print(f'  any pitch : radius '
              f'{any_["radius"][0] * 1000:.0f}-{any_["radius"][1] * 1000:.0f} mm, '
              f'height {any_["height"][0] * 1000:+.0f} to '
              f'{any_["height"][1] * 1000:+.0f} mm')
        if at:
            share = 100.0 * at['cells'] / any_['cells']
            print(f'  {share:.0f}% of the reachable area can be picked from '
                  f'at {args.pitch_deg:.0f} deg')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())

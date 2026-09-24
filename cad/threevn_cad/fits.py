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
Every dimension where two parts meet, defined once.

WHY THIS MODULE EXISTS. Both sides of a joint were sized independently
and both looked reasonable:

    beam pivot bore   3.35 mm   (an M3 screw plus clearance)
    bushing outside   5.75 mm   (an M3 bore plus two walls)

The bushing was 2.4 mm too big to enter the hole it was made for. The
same mistake sat at the pan joint: the base's bearing seat came out at
48.8 mm outside diameter and the turret's bore at 44.7 mm, a 4.1 mm
interference.

Neither is a hard error. Both parts export, both pass every geometric
check, both look right, and the mistake is found with the parts in your
hands.

The cause was sizing each hole against the NOMINAL FEATURE - a screw, a
link radius - rather than against the part that actually goes into it.
So the mating dimensions live here, each is computed once, and both
sides of every joint call the same function. A hole and the thing that
enters it cannot disagree if only one of them is a decision.
"""

from threevn_cad import profile as prof


def screw_bore_radius(cfg):
    """
    A hole an M3 screw passes through.

    FDM holes print undersize, so a nominal 3.0 mm hole takes an M3 only
    with help from a drill.
    """
    fab = prof.fabrication(cfg)
    return (fab['m3_hole_mm'] + fab['hole_clearance_mm']) / 2.0


def bushing_inner_radius(cfg):
    """The bushing's bore: a sliding fit on an M3 screw."""
    return screw_bore_radius(cfg)


def bushing_outer_radius(cfg):
    """
    The bushing's outside: an interference fit in its housing.

    NO clearance, deliberately. This is the one surface in the assembly
    meant to be pressed rather than dropped in - a bushing that spins in
    its bore wears the bracket instead of itself, which is the opposite
    of the point.
    """
    fab = prof.fabrication(cfg)
    return bushing_inner_radius(cfg) + fab['min_wall_mm']


def bushing_flange_radius(cfg):
    """The lip that stops the bushing pushing all the way through."""
    fab = prof.fabrication(cfg)
    return bushing_outer_radius(cfg) + fab['min_wall_mm']


def pivot_bore_radius(cfg, bushed=True):
    """
    A pivot hole in a structural part.

    `bushed` is the whole point of this function. A bushed pivot has to
    admit the BUSHING, which is two wall thicknesses larger than the
    screw - and sizing it to the screw is exactly the bug this module
    was written for.
    """
    if not bushed:
        return screw_bore_radius(cfg)
    fab = prof.fabrication(cfg)
    return bushing_outer_radius(cfg) + fab['hole_clearance_mm']


def bearing_seat_outer_radius(cfg):
    """
    The boss on the base plate that the turret spins on.

    Sized from the turret's nominal radius, which is what the URDF's
    collision shape describes.
    """
    fab = prof.fabrication(cfg)
    turret = prof.link_mm(cfg, 'shoulder_link')
    return turret['radius'] + 2 * fab['min_wall_mm']


def bearing_seat_inner_radius(cfg):
    """The shaft clearance through the middle of the seat."""
    fab = prof.fabrication(cfg)
    turret = prof.link_mm(cfg, 'shoulder_link')
    return turret['radius'] + fab['hole_clearance_mm']


def turret_bore_radius(cfg):
    """
    The hole in the turret that drops over the bearing seat.

    Derived from the SEAT, not from the turret's own nominal radius.
    Taking the nominal radius gave a 4.1 mm interference, because the
    seat is a boss around that radius rather than the radius itself.
    """
    fab = prof.fabrication(cfg)
    return bearing_seat_outer_radius(cfg) + fab['hole_clearance_mm']


def washer_bore_radius(cfg):
    """The thrust washer drops over the same seat the turret does."""
    fab = prof.fabrication(cfg)
    return bearing_seat_outer_radius(cfg) + fab['hole_clearance_mm']


def washer_outer_radius(cfg):
    """Wide enough to carry the turret's face, narrow enough to stay hidden."""
    fab = prof.fabrication(cfg)
    return turret_bore_radius(cfg) + 2 * fab['min_wall_mm']


#: Every fit in the assembly, as (hole, plug, label). The tests walk
#: this so a new joint cannot be added without its clearance being
#: checked - the failure mode here was never a wrong formula, it was two
#: formulas nobody compared.
MATING_PAIRS = (
    (pivot_bore_radius, bushing_outer_radius, 'bushing in a pivot bore'),
    (turret_bore_radius, bearing_seat_outer_radius,
     'turret over the bearing seat'),
    (washer_bore_radius, bearing_seat_outer_radius,
     'thrust washer over the bearing seat'),
    (bushing_inner_radius, None, 'M3 screw through a bushing'),
)

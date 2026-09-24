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
What each link actually weighs, from the geometry that will be printed.

The arm's link masses were guesses for fifteen phases. They had to be:
nothing had been designed, so there was nothing to weigh or compute, and
`provenance: estimated` said exactly that.

They were wrong in BOTH DIRECTIONS - base_link 2.3x too heavy at 250 g,
shoulder_link 40% too light at 60 g - which is worse than being
uniformly wrong. A tipping margin computed from them could have been
optimistic or pessimistic and nobody could tell which.

A link's mass is its printed structure, plus the servo that RIDES on it,
plus that servo's horn adapter. Which servo rides where is not a choice
made here: it is the parent of each joint in the profile, and direct
drive means the servo sits at the joint it drives.

STILL NOT MEASURED. PLA density is nominal, the servo masses are
datasheet figures, and fasteners are not counted. `computed` is better
than a guess and worse than a scale.
"""

#: Density of PLA, g/cm3. Nominal: real filament varies by a few percent
#: and infill settings by much more, so this is the figure a slicer
#: would quote, not one anybody weighed.
PLA_DENSITY = 1.24

#: Which printed part provides each link's structure.
STRUCTURE = {
    'base_link': 'base_plate',
    'shoulder_link': 'shoulder_turret',
    'upper_arm_link': 'upper_arm_link',
    'forearm_link': 'forearm_link',
    'wrist_link': 'wrist_bracket',
    'gripper_base_link': 'gripper_base',
    'gripper_left_finger_link': 'gripper_finger',
    'gripper_right_finger_link': 'gripper_finger',
}


def _grams(solid):
    """Convert a solid's volume to grams of PLA."""
    return solid.volume / 1000.0 * PLA_DENSITY


def servo_riders(cfg):
    """
    Return {link: [servo, ...]} from the joints, not from a hand list.

    A servo rides on the PARENT of the joint it drives. Reading that
    from the profile means re-parenting the arm moves the mass with it,
    and a hand-maintained table would not.
    """
    riders = {}
    for joint in cfg['joints'].values():
        if 'servo' in joint:
            riders.setdefault(joint['parent'], []).append(joint['servo'])
    return riders


def compute(cfg, mechanism='direct'):
    """Return {link: mass_kg} for every link the CAD can account for."""
    from threevn_cad import parts as parts_mod

    available = parts_mod.parts_for(mechanism)
    riders = servo_riders(cfg)
    servo_masses = {name: spec['mass_kg'] * 1000.0
                    for name, spec in cfg['servos'].items()}

    out = {}
    for link, part_name in STRUCTURE.items():
        if part_name not in available or link not in cfg['links']:
            continue
        grams = _grams(available[part_name](cfg, mechanism=mechanism))

        for servo in riders.get(link, []):
            grams += servo_masses[servo]
            # Each driven joint needs an adapter on its servo's horn.
            adapter = f'horn_adapter_{servo}'
            if adapter in available:
                grams += _grams(available[adapter](cfg, mechanism=mechanism))

        out[link] = round(grams / 1000.0, 4)
    return out


def compare(cfg, mechanism='direct'):
    """Return [(link, declared_kg, computed_kg)] for every accounted link."""
    computed = compute(cfg, mechanism)
    return [(link, cfg['links'][link]['mass'], value)
            for link, value in sorted(computed.items())]

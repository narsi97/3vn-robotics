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

"""The one unit conversion in the CAD, and the data it depends on."""

import pytest
from threevn_cad import profile as prof


@pytest.fixture(scope='module')
def cfg():
    """The real robot profile, not a fixture: the CAD reads what ships."""
    return prof.load()


def test_metres_become_millimetres_exactly_once(cfg):
    """
    THE CONVERSION THAT ONLY HAPPENS IN ONE PLACE.

    The URDF is in metres; every printer, datasheet and M3 screw is in
    millimetres. A part 1000x too large still exports, still opens, and
    still looks correct until someone checks the scale bar.
    """
    assert prof.mm(0.120) == pytest.approx(120.0)
    link = prof.link_mm(cfg, 'upper_arm_link')
    # A desktop arm link is tens of millimetres, not tens of metres.
    assert 20.0 < link['z'] < 400.0, (
        f'upper_arm_link is {link["z"]} mm long, which is not a desktop arm'
    )


def test_a_cylinder_is_not_returned_as_a_box(cfg):
    """
    Shape is reported, not assumed.

    A cylinder unpacked as a box silently becomes a very strange
    bracket, and the mistake is invisible in the numbers.
    """
    shoulder = prof.link_mm(cfg, 'shoulder_link')
    assert shoulder['type'] in ('box', 'cylinder')
    if shoulder['type'] == 'cylinder':
        assert 'radius' in shoulder and 'x' not in shoulder


def test_an_unknown_geometry_type_is_refused(cfg):
    """Better than producing a part from whatever keys happened to exist."""
    broken = {'links': {'odd': {'geometry': {'type': 'torus'}}}}
    with pytest.raises(ValueError, match='unsupported geometry'):
        prof.link_mm(broken, 'odd')


def test_every_servo_has_a_physical_envelope(cfg):
    """
    Torque and mass are not enough to cut a pocket.

    A servo declared for a joint but with no body dimensions means the
    CAD cannot hold it, and the failure should be here rather than at
    the printer.
    """
    for name in cfg['servos']:
        body = prof.servo(cfg, name)
        for key in ('length_mm', 'width_mm', 'height_mm',
                    'mount_hole_spacing_mm', 'shaft_offset_mm'):
            assert key in body, f'{name} is missing {key}'
            assert body[key] > 0


def test_servo_dimensions_are_in_millimetres_already(cfg):
    """
    The datasheet block is NOT converted, deliberately.

    Putting a unit conversion between a datasheet and the pocket that
    has to hold the part is how a servo ends up not fitting for a reason
    invisible in the drawing.
    """
    body = prof.servo(cfg, 'mg996r')
    # An MG996R is about 40 mm long. If this were metres it would be 0.04.
    assert 30.0 < body['length_mm'] < 60.0


def test_the_mounting_flange_is_wider_than_the_body(cfg):
    """
    A sanity check on the datasheet transcription.

    The screw flange has to overhang the case or the screws would pass
    through the servo. Getting these two swapped produces a pocket the
    servo cannot enter.
    """
    for name in ('mg996r', 'sg90'):
        body = prof.servo(cfg, name)
        assert body['flange_length_mm'] > body['length_mm'], name
        assert body['mount_hole_spacing_mm'] > body['length_mm'], name


def test_every_actuated_joint_names_a_servo_the_cad_knows(cfg):
    """A joint driven by a servo with no envelope cannot be built."""
    for joint, spec in cfg['joints'].items():
        if 'servo' not in spec:
            continue
        prof.servo(cfg, spec['servo'])


def test_fabrication_constants_are_present_and_plausible(cfg):
    """
    They describe the PRINTER, not the robot.

    Separated for that reason: the same arm on a better machine wants
    different clearances and nothing about the kinematics changes.
    """
    fab = prof.fabrication(cfg)
    assert fab['min_wall_mm'] >= 3 * fab['nozzle_mm'] * 0.9, (
        'a wall thinner than about three perimeters does not survive a '
        'servo stalling against it'
    )
    assert 0.0 < fab['hole_clearance_mm'] < 1.0
    assert 30.0 <= fab['max_overhang_deg'] <= 60.0


def test_a_profile_without_fabrication_is_refused():
    """The CAD cannot guess wall thickness."""
    with pytest.raises(KeyError, match='fabrication'):
        prof.fabrication({'links': {}})


def test_a_missing_profile_says_why_it_matters():
    """
    The error explains the design, not just the path.

    'File not found' invites someone to point it at a copy; this says
    why it must be the same file the URDF reads.
    """
    with pytest.raises(FileNotFoundError, match='SAME profile'):
        prof.load('/nonexistent/profile.yaml')

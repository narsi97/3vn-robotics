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
Inertia tensors describe physically possible rigid bodies.

An invalid inertia tensor does not produce an error at load time. It
produces NaN the first time a contact force is applied, which surfaces as
"the robot exploded on spawn" or "Gazebo went unstable" -- symptoms that
look like simulator bugs and cost hours. Catching it here, with no
simulator running, is the cheapest possible place.
"""
from conftest import PROFILES
import numpy as np
import pytest
from urdf_parser_py.urdf import URDF


def _tensor(inertia):
    return np.array([
        [inertia.ixx, inertia.ixy, inertia.ixz],
        [inertia.ixy, inertia.iyy, inertia.iyz],
        [inertia.ixz, inertia.iyz, inertia.izz],
    ])


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_masses_are_positive(expanded, profile):
    for link in URDF.from_xml_string(expanded(profile=profile)).links:
        if link.inertial is None:
            continue
        assert link.inertial.mass > 0, f'{link.name}: mass {link.inertial.mass} <= 0'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_inertia_is_symmetric_and_positive_definite(expanded, profile):
    for link in URDF.from_xml_string(expanded(profile=profile)).links:
        if link.inertial is None:
            continue
        inertia_tensor = _tensor(link.inertial.inertia)
        assert np.allclose(inertia_tensor, inertia_tensor.T), \
            f'{link.name}: inertia tensor not symmetric'
        eigenvalues = np.linalg.eigvalsh(inertia_tensor)
        assert (eigenvalues > 0).all(), \
            f'{link.name}: not positive definite, eigenvalues {eigenvalues}'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_principal_moments_satisfy_triangle_inequality(expanded, profile):
    """
    For any real rigid body the principal moments obey.

    I1 + I2 >= I3 for every permutation. A tensor that violates this
    describes no physical object, and the physics engine will eventually
    say so in NaN.
    """
    for link in URDF.from_xml_string(expanded(profile=profile)).links:
        if link.inertial is None:
            continue
        a, b, c = sorted(np.linalg.eigvalsh(_tensor(link.inertial.inertia)))
        assert a + b >= c * (1 - 1e-9), (
            f'{link.name}: principal moments {a:.3e}, {b:.3e}, {c:.3e} '
            'violate the triangle inequality'
        )


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_total_mass_is_plausible(expanded, profile):
    """
    A desktop arm of hobby servos, PLA and M3 hardware.

    Outside this band something is wrong by an order of magnitude, which
    is the single most common URDF error and the one most likely to make
    a simulation behave bizarrely rather than fail.
    """
    total = sum(
        link.inertial.mass
        for link in URDF.from_xml_string(expanded(profile=profile)).links
        if link.inertial
    )
    assert 0.2 < total < 3.0, f'total mass {total:.3f} kg is not a desktop arm'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_inertia_respects_the_configured_floor(expanded, profile):
    """
    The configured inertia floor is actually applied.

    `min_inertia` exists to stop near-zero values producing NaN under
    contact.
    """
    import yaml

    floor = yaml.safe_load(profile.read_text())['defaults']['min_inertia']
    for link in URDF.from_xml_string(expanded(profile=profile)).links:
        if link.inertial is None:
            continue
        inertia = link.inertial.inertia
        for axis, value in (('ixx', inertia.ixx), ('iyy', inertia.iyy), ('izz', inertia.izz)):
            assert value >= floor * (1 - 1e-9), \
                f'{link.name}.{axis} = {value:.3e} is below the floor {floor:.3e}'

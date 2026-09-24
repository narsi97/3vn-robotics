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
The YAML obeys its own conventions.

Cheap discipline that directly implements the spec's requirement to
"isolate unknowns into configuration": if a number is a guess, the file
must say so, and units must be visible in the key name rather than
remembered.
"""
from conftest import (ALL_PROFILES, COMPONENT_KINDS, COMPONENT_PROFILES,
                      COMPOSED_PROFILES, ENTRY_POINTS, MASS_BAND,
                      REQUIRED_FRAMES)
import pytest
import yaml

#: `computed` joined the set when the CAD produced real geometry: a
#: mass derived from a modelled solid plus a datasheet servo mass is
#: better than a guess and worse than a scale. Keeping it distinct from
#: `measured` is the point - nothing here has been weighed.
VALID_PROVENANCE = {'measured', 'datasheet', 'computed', 'estimated'}


def _cfg(profile):
    return yaml.safe_load(profile.read_text())


@pytest.mark.parametrize('profile', COMPONENT_PROFILES, ids=lambda p: p.stem)
def test_every_mass_declares_provenance(profile):
    """
    Where a number came from is part of the number.

    The arm's link masses were guesses until the CAD existed; they are
    now `computed` from modelled geometry plus datasheet servo masses.
    That distinction has to be visible in the data rather than implied
    by a comment, because nothing here has been weighed and the
    difference decides how much to trust a tipping margin.
    """
    for name, spec in _cfg(profile)['links'].items():
        assert 'provenance' in spec, f'{name}: no provenance declared'
        assert spec['provenance'] in VALID_PROVENANCE, (
            f'{name}: provenance {spec["provenance"]!r} not one of '
            f'{sorted(VALID_PROVENANCE)}'
        )


@pytest.mark.parametrize('profile', COMPONENT_PROFILES, ids=lambda p: p.stem)
def test_revolute_joints_use_degree_keys(profile):
    """
    Revolute limits declare their units in the key name.

    A revolute limit written as `lower` instead of `lower_deg` would be
    silently interpreted as radians.
    """
    for name, spec in _cfg(profile)['joints'].items():
        if spec['type'] != 'revolute':
            continue
        assert 'lower_deg' in spec['limit'], f'{name}: use lower_deg, not lower'
        assert 'upper_deg' in spec['limit'], f'{name}: use upper_deg, not upper'
        assert 'initial_position_deg' in spec, f'{name}: use initial_position_deg'


@pytest.mark.parametrize('profile', COMPONENT_PROFILES, ids=lambda p: p.stem)
def test_prismatic_joints_use_metre_keys(profile):
    for name, spec in _cfg(profile)['joints'].items():
        if spec['type'] != 'prismatic':
            continue
        assert 'lower' in spec['limit'] and 'upper' in spec['limit'], \
            f'{name}: prismatic limits are metres, use lower/upper'
        assert 'lower_deg' not in spec['limit'], f'{name}: prismatic limit in degrees?'


@pytest.mark.parametrize('profile', COMPONENT_PROFILES, ids=lambda p: p.stem)
def test_no_dimension_is_zero_or_negative(profile):
    for name, spec in _cfg(profile)['links'].items():
        for key, value in spec['geometry'].items():
            if key == 'type':
                continue
            assert value > 0, f'{name}.geometry.{key} = {value} must be positive'


@pytest.mark.parametrize('profile', COMPONENT_PROFILES, ids=lambda p: p.stem)
def test_every_joint_references_declared_links(profile):
    """
    Every joint references a link that is actually declared.

    A joint pointing at a link that does not exist produces a confusing
    parse error much later.
    """
    cfg = _cfg(profile)
    # A base has no `frames` block -- its links ARE its frames.
    known = set(cfg['links']) | set(cfg.get('frames', {}))
    for name, spec in cfg['joints'].items():
        # A base declares wheel POSITIONS and lets the macro own the
        # topology -- four wheels on a chassis is fixed, so writing
        # parent/child per wheel would be noise. Only joints that
        # declare a parent are checked against declared links.
        if 'parent' not in spec:
            continue
        assert spec['parent'] in known, f'{name}: unknown parent {spec["parent"]!r}'
        assert spec['child'] in known, f'{name}: unknown child {spec["child"]!r}'


@pytest.mark.parametrize('profile', COMPONENT_PROFILES, ids=lambda p: p.stem)
def test_model_name_is_ros_legal(profile):
    """
    REP-144: an identifier that may become a node, topic or namespace.

    token must start with a letter and contain only lowercase
    alphanumerics and underscores. `3vn_arm_v1` would pass YAML parsing
    and then fail inside DDS with an opaque error.
    """
    import re

    model = _cfg(profile)['meta']['model']
    assert re.fullmatch(r'[a-z][a-z0-9_]*', model), (
        f'model {model!r} is not a legal ROS identifier -- it must start '
        'with a letter (see docs/decisions/0001-package-naming.md)'
    )


@pytest.mark.parametrize('profile', COMPONENT_PROFILES, ids=lambda p: p.stem)
def test_mimic_targets_an_existing_joint(profile):
    cfg = _cfg(profile)
    for name, spec in cfg['joints'].items():
        if 'mimic' not in spec:
            continue
        target = spec['mimic']['joint']
        assert target in cfg['joints'], f'{name}: mimics unknown joint {target!r}'
        assert target != name, f'{name}: mimics itself'


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
def test_every_profile_declares_a_known_family(profile):
    """
    Every profile names a robot family the harness knows how to expand.

    This is the guard on the other side of `meta.kind`. Scoping the suite
    by family stops an arm profile being validated against base rules --
    but it introduces the opposite failure, where a profile with an
    unrecognised kind is quietly expanded by nothing and tested by
    nothing. A new family must be a deliberate act: add it to
    ENTRY_POINTS, MASS_BAND and REQUIRED_FRAMES, or this fails.
    """
    import yaml

    kind = yaml.safe_load(profile.read_text())['meta']['kind']
    assert kind in ENTRY_POINTS, (
        f'{profile.name} declares kind={kind!r}, which has no xacro entry '
        f'point. Known: {sorted(ENTRY_POINTS)}'
    )
    assert kind in MASS_BAND, f'kind={kind!r} has no mass band'
    assert kind in REQUIRED_FRAMES, f'kind={kind!r} has no required frames'


# -- composition profiles ----------------------------------------------
#
# A composition declares no links, joints or masses. Its whole job is to
# name components and say where they meet, so what is worth asserting is
# the opposite of the component schema: that it stays empty of physical
# numbers.

@pytest.mark.parametrize('profile', COMPOSED_PROFILES, ids=lambda p: p.stem)
def test_composition_names_components_that_exist(profile):
    """Every named component resolves to a real profile."""
    import yaml

    cfg = yaml.safe_load(profile.read_text())
    parts = cfg.get('components')
    assert parts, f'{profile.name}: a composition must name its components'
    for role, name in parts.items():
        assert (profile.parent / name).is_file(), (
            f'{profile.name}: component {role}={name!r} does not exist'
        )


@pytest.mark.parametrize('profile', COMPOSED_PROFILES, ids=lambda p: p.stem)
def test_composition_components_are_components(profile):
    """
    A composition is built from component families, not other compositions.

    Nesting would be defensible eventually, but nothing supports it today
    -- the entry point loads each component's YAML directly and would
    quietly build nothing from a profile that has no links.
    """
    import yaml

    cfg = yaml.safe_load(profile.read_text())
    for role, name in cfg['components'].items():
        kind = yaml.safe_load((profile.parent / name).read_text())['meta']['kind']
        assert kind in COMPONENT_KINDS, (
            f'{profile.name}: component {role} has kind {kind!r}, which is '
            f'not a component family {sorted(COMPONENT_KINDS)}'
        )


@pytest.mark.parametrize('profile', COMPOSED_PROFILES, ids=lambda p: p.stem)
def test_composition_declares_no_physical_numbers(profile):
    """
    THE SINGLE SOURCE OF TRUTH, ENFORCED.

    The moment a composition carries its own `links` or `servos`, the
    same quantity exists in two files and one of them is stale. Every
    dimension belongs to the component that owns it; this file may say
    only which components and where they join.
    """
    import yaml

    cfg = yaml.safe_load(profile.read_text())
    owned_elsewhere = {'links', 'joints', 'servos', 'motors', 'frames',
                       'sensors', 'limit_derate', 'defaults'}
    present = owned_elsewhere & set(cfg)
    assert not present, (
        f'{profile.name} declares {sorted(present)}, which belong to the '
        f'component profiles. Two sources of truth is one too many.'
    )


@pytest.mark.parametrize('profile', COMPOSED_PROFILES, ids=lambda p: p.stem)
def test_composition_mount_is_fully_specified(profile):
    """
    The mount says everything needed to place one component on another.

    Except the vertical offset, which is derived from the arm's own
    mount_plate_link -- see the note in threevn_mm_v1.yaml. A `z` here
    would be exactly the second source of truth the test above forbids,
    so its ABSENCE is asserted rather than its value.
    """
    import yaml

    mount = yaml.safe_load(profile.read_text()).get('mount')
    assert mount, f'{profile.name}: no mount block'
    for key in ('prefix', 'x', 'y', 'yaw'):
        assert key in mount, f'{profile.name}: mount is missing {key!r}'
    assert mount['prefix'].endswith('_'), (
        f'{profile.name}: prefix {mount["prefix"]!r} must end in an '
        f'underscore, or it runs into the link name'
    )
    assert 'z' not in mount, (
        f'{profile.name}: mount.z is derived from the arm profile, not '
        f'declared here'
    )

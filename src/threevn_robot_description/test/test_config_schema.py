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
from conftest import PROFILES
import pytest
import yaml

VALID_PROVENANCE = {'measured', 'datasheet', 'estimated'}


def _cfg(profile):
    return yaml.safe_load(profile.read_text())


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_every_mass_declares_provenance(profile):
    """
    Right now every value in this file is an engineer's guess. That.

    should be visible in the data, not implied by a comment, so it can be
    audited and re-measured after fabrication.
    """
    for name, spec in _cfg(profile)['links'].items():
        assert 'provenance' in spec, f'{name}: no provenance declared'
        assert spec['provenance'] in VALID_PROVENANCE, (
            f'{name}: provenance {spec["provenance"]!r} not one of '
            f'{sorted(VALID_PROVENANCE)}'
        )


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
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


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_prismatic_joints_use_metre_keys(profile):
    for name, spec in _cfg(profile)['joints'].items():
        if spec['type'] != 'prismatic':
            continue
        assert 'lower' in spec['limit'] and 'upper' in spec['limit'], \
            f'{name}: prismatic limits are metres, use lower/upper'
        assert 'lower_deg' not in spec['limit'], f'{name}: prismatic limit in degrees?'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_no_dimension_is_zero_or_negative(profile):
    for name, spec in _cfg(profile)['links'].items():
        for key, value in spec['geometry'].items():
            if key == 'type':
                continue
            assert value > 0, f'{name}.geometry.{key} = {value} must be positive'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_every_joint_references_declared_links(profile):
    """
    Every joint references a link that is actually declared.

    A joint pointing at a link that does not exist produces a confusing
    parse error much later.
    """
    cfg = _cfg(profile)
    known = set(cfg['links']) | set(cfg['frames'])
    for name, spec in cfg['joints'].items():
        assert spec['parent'] in known, f'{name}: unknown parent {spec["parent"]!r}'
        assert spec['child'] in known, f'{name}: unknown child {spec["child"]!r}'


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
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


@pytest.mark.parametrize('profile', PROFILES, ids=lambda p: p.stem)
def test_mimic_targets_an_existing_joint(profile):
    cfg = _cfg(profile)
    for name, spec in cfg['joints'].items():
        if 'mimic' not in spec:
            continue
        target = spec['mimic']['joint']
        assert target in cfg['joints'], f'{name}: mimics unknown joint {target!r}'
        assert target != name, f'{name}: mimics itself'

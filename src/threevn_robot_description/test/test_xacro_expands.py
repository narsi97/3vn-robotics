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
The description expands for every profile and every hardware target.

This is the widest, cheapest net in the suite: it catches YAML key typos,
xacro syntax errors and macro-arity mistakes across the full matrix before
any more specific test gets a chance to fail confusingly.
"""
from conftest import ALL_PROFILES, TARGETS
import pytest


@pytest.mark.parametrize('profile', ALL_PROFILES, ids=lambda p: p.stem)
@pytest.mark.parametrize('target', TARGETS)
def test_expands_without_error(expanded, profile, target):
    xml = expanded(profile=profile, target=target)
    assert xml.lstrip().startswith('<?xml')
    assert '<robot' in xml


def test_prefix_is_applied_to_every_link(expanded):
    """
    The prefix argument reaches every link.

    Without this, a second arm instance would collide with the first.
    Phase 11 (mobile manipulator) depends on it.
    """
    import xml.etree.ElementTree as ET

    root = ET.fromstring(expanded(prefix='left_'))
    names = [link.get('name') for link in root.iter('link')]
    assert names, 'no links produced'
    unprefixed = [n for n in names if not n.startswith('left_')]
    assert not unprefixed, f'links ignored the prefix: {unprefixed}'

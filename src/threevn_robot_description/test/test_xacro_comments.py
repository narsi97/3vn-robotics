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
Double hyphens inside XML comments.

XML forbids `--` inside a comment. This project writes long explanatory
comments in prose, where an em dash is the natural punctuation, so the
mistake is easy to make and has now been made in three separate phases.

What makes it worth its own test is the error. Xacro reports:

    not well-formed (invalid token): line 66, column 65

with no mention of comments, no mention of hyphens, and a line number
in the EXPANDED document that does not correspond to the file anyone
edited. Every occurrence has cost real time. This test names the file,
the line, and the reason.
"""

import pathlib
import re

import pytest


def _repo_root():
    """Walk up to the workspace, so this works from any cwd."""
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'VERSION').is_file():
            return parent
    return None


ROOT = _repo_root()
XACROS = sorted(ROOT.rglob('src/**/*.xacro')) if ROOT else []
WORLDS = sorted(ROOT.rglob('src/**/*.sdf')) if ROOT else []
MARKUP = XACROS + WORLDS


@pytest.mark.skipif(not MARKUP, reason='running outside a source checkout')
@pytest.mark.parametrize('path', MARKUP, ids=lambda p: p.name)
def test_no_double_hyphen_inside_a_comment(path):
    """A comment containing `--` makes the whole document unparseable."""
    text = path.read_text()
    offenders = []
    for match in re.finditer(r'<!--.*?-->', text, re.S):
        body = match.group(0)[4:-3]
        if '--' in body:
            line = text[:match.start()].count('\n') + 1
            snippet = next((ln.strip() for ln in body.splitlines()
                            if '--' in ln), '')
            offenders.append(f'line {line}: {snippet!r}')

    assert not offenders, (
        f'{path.name} has `--` inside an XML comment, which is illegal and '
        f'reports as "not well-formed (invalid token)" pointing at the '
        f'EXPANDED document rather than this file. Use a single hyphen or '
        f'a comma.\n  ' + '\n  '.join(offenders)
    )

#!/usr/bin/env python3
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
Check that terms.yaml decides every term in the course glossary, once.

    make terms-check

The glossary is the `concepts` of every lesson in course/*.yaml.
terms.yaml sorts terms into three lists: names that are never
translated, vocabulary kept in English, and ideas that are translated.

Two things go wrong quietly without this. A lesson gains a glossary
term and nobody decides how a dubbed recording should say it, so each
translation decides for itself. Or a term ends up in two lists, and
which rule applies depends on which list a tool reads first.

Exit code 0 only if every glossary term is in exactly one list.
"""

import pathlib
import sys

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
TERMS = ROOT / 'recordings' / 'terms.yaml'
LISTS = ('never_translate', 'keep_in_english', 'translate')


def glossary():
    """Return every glossary term in the course, with its lesson id."""
    found = {}
    for path in sorted((ROOT / 'course').glob('*.yaml')):
        stage = yaml.safe_load(path.read_text()) or {}
        for lesson in stage.get('lessons') or []:
            for concept in lesson.get('concepts') or []:
                found[concept['term']] = lesson['id']
    return found


def classified():
    """Return {list name: set of lower-cased terms} from terms.yaml."""
    doc = yaml.safe_load(TERMS.read_text())
    result = {}
    for name in LISTS:
        entries = doc.get(name) or []
        result[name] = {
            (entry['term'] if isinstance(entry, dict) else entry).lower()
            for entry in entries
        }
    return result


def main():
    """Report unclassified and doubly classified terms."""
    lists = classified()
    errors = []

    for index, first in enumerate(LISTS):
        for second in LISTS[index + 1:]:
            for term in sorted(lists[first] & lists[second]):
                errors.append(
                    f'{term!r} is in both `{first}` and `{second}`')

    everything = set().union(*lists.values())
    terms = glossary()
    for term, lesson in sorted(terms.items()):
        if term.lower() not in everything:
            errors.append(
                f'{term!r} (lesson {lesson}) is in the course glossary '
                f'and in no list')

    if errors:
        print(f'  recordings/terms.yaml: {len(errors)} problem(s)')
        for error in errors:
            print(f'    - {error}')
        return 1
    sizes = ', '.join(f'{len(lists[name])} {name}' for name in LISTS)
    print(f'  terms ok: {len(terms)} glossary terms decided ({sizes})')
    return 0


if __name__ == '__main__':
    sys.exit(main())

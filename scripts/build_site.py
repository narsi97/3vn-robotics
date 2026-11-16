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
Render the course, and this repository's documentation, as a static site.

    make site            # validate, then render into site/
    make course-check    # validate only

The curriculum is DATA in course/*.yaml. The docs underneath are the
project's own, rendered as the reference the lessons link into -- not a
second copy of the explanation, which would drift from the first within
a week.

The curriculum is VALIDATED before anything is rendered, and a failure
stops the build. A lesson that tells a student to run `make foo` when
the Makefile has no `foo`, or to read a doc that has been renamed, is a
broken lesson that looks finished. Every error is reported at once, not
the first: a curriculum with three problems should be fixed in one pass.

Every lesson carries topic TAGS from the fixed list in course/tags.yaml.
Each tag gets a page (tags/<slug>/index.html) listing every lesson that
carries it, across the whole syllabus, and what to read alongside it.
Those pages are the only ones below the site root; their links are
written as if they were at the root and re-rooted on the way out.

A lesson about something else can also MENTION a topic by quoting the
one passage that touches it. The topic page links straight to that
passage with a URL text fragment, and validation fails if the quote is
no longer in the lesson.

Three constraints come from where this is served -- behind the shared
3VNSYSTEMS Caddy, on an unlisted path:

  * Every link is RELATIVE. The site lives under /rc-<something>/ today
    and may live at a bare domain tomorrow, and that move is supposed to
    be a config change rather than a rewrite.
  * No JavaScript at all. The enforced CSP is `script-src 'self'` with
    no 'unsafe-inline', so an inline <script> is dead on arrival. The
    answers to self-check questions use <details>, which needs none.
    Styles are inline because `style-src` does allow 'unsafe-inline'.
  * Every page carries noindex, and robots.txt disallows everything.
    Unlisted means unlisted; this is obscurity, not security, exactly as
    projects-for-businesses/README.md says of the client sites.
"""

import argparse
import datetime
import html as htmllib
import os
import pathlib
import re
import shutil
import subprocess
import sys
import urllib.parse

import yaml

#: The reading order for the reference section. Anything in docs/ not
#: named here is still rendered, under "Everything else" -- so adding a
#: doc never means it silently fails to appear.
CURRICULUM = [
    ('Start here', [
        ('getting-started.md', 'Getting started'),
        ('learning-path.md', 'Learning it without a robot'),
        ('viewing.md', 'Three ways to watch'),
    ]),
    ('The system', [
        ('architecture.md', 'Architecture'),
        ('ros2.md', 'ROS 2'),
        ('simulation.md', 'Simulation'),
        ('gazebo.md', 'Gazebo'),
        ('manipulation.md', 'Manipulation: what the arm cannot do'),
        ('mobile-base.md', 'Mobile base'),
        ('mobile-manipulator.md', 'Mobile manipulator'),
        ('perception.md', 'Perception'),
        ('navigation.md', 'Navigation'),
    ]),
    ('Engineering practice', [
        ('development.md', 'Development'),
        ('testing.md', 'Testing'),
        ('cicd.md', 'CI/CD'),
        ('docker.md', 'Docker'),
        ('deployment.md', 'Deployment'),
        ('dashboard.md', 'Dashboard and health'),
        ('monitoring.md', 'Monitoring'),
        ('versioning.md', 'Versioning'),
        ('troubleshooting.md', 'Troubleshooting'),
    ]),
    ('Data and models', [
        ('data-pipeline.md', 'Data pipeline'),
        ('machine-learning.md', 'Machine learning'),
        ('mlops.md', 'MLOps'),
    ]),
    ('Hardware', [
        ('cad.md', 'CAD'),
        ('bom-hardware.md', 'Bill of materials'),
        ('hardware.md', 'Hardware'),
        ('firmware.md', 'Firmware'),
        ('safety.md', 'Safety'),
        ('so101-spike.md', 'Spike: a robot we did not design'),
        ('so101.md', 'The SO-101, as a second robot'),
    ]),
]

#: What a stage, lesson and project must carry. A field left out here is
#: a field some page renders as a hole.
STAGE_FIELDS = ('id', 'number', 'kind', 'title', 'price', 'summary',
                'outcomes', 'lessons', 'project')
LESSON_FIELDS = ('id', 'title', 'tags', 'minutes', 'objective', 'body',
                 'steps', 'checkpoint')
PROJECT_FIELDS = ('title', 'minutes', 'brief', 'tasks', 'done_when')
TAG_FIELDS = ('slug', 'label', 'description')

#: The topic vocabulary, in course/ beside the stages but not one of them.
TAGS_FILE = 'tags.yaml'

#: How many "often studied alongside" lessons a topic page recommends,
#: after every lesson its own lessons build on.
RELATED_PICKS = 4

STYLE = """
:root {
  --bg: #11131a; --panel: #181b24; --line: #272b36;
  --ink: #e6e8ef; --dim: #9aa0b0; --accent: #7fb7ff; --warn: #ffcc66;
  --good: #7ddc9a; --bad: #ff8a80;
}
* { box-sizing: border-box; }
body {
  margin: 0; background: var(--bg); color: var(--ink);
  font: 16px/1.65 ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
}
.wrap { max-width: 860px; margin: 0 auto; padding: 0 20px 72px; }
header.site { border-bottom: 1px solid var(--line); margin-bottom: 36px; }
header.site .wrap { padding-top: 26px; padding-bottom: 22px; }
.brand { font-weight: 650; letter-spacing: .02em; }
.brand a { color: var(--ink); text-decoration: none; }
.meta { color: var(--dim); font-size: 13px; margin-top: 6px; }
.badge {
  display: inline-block; padding: 2px 9px; border: 1px solid var(--line);
  border-radius: 999px; font-size: 12px; color: var(--dim); margin-left: 6px;
}
a { color: var(--accent); }
h1 { font-size: 28px; line-height: 1.25; margin: 0 0 14px; }
h2 { font-size: 20px; margin: 38px 0 12px; }
h3 { font-size: 16px; margin: 24px 0 8px; color: var(--dim);
     text-transform: uppercase; letter-spacing: .06em; }
p, li { color: var(--ink); }
code {
  background: var(--panel); border: 1px solid var(--line); border-radius: 4px;
  padding: 1px 5px; font-size: 13.5px;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}
pre {
  background: var(--panel); border: 1px solid var(--line); border-radius: 8px;
  padding: 14px 16px; overflow-x: auto;
}
pre code { background: none; border: 0; padding: 0; font-size: 13px; }
table { border-collapse: collapse; width: 100%; margin: 16px 0; font-size: 14.5px;
        display: block; overflow-x: auto; }
th, td { border: 1px solid var(--line); padding: 7px 10px; text-align: left;
         vertical-align: top; }
th { background: var(--panel); }
blockquote {
  margin: 16px 0; padding: 10px 16px; border-left: 3px solid var(--line);
  color: var(--dim);
}
.note {
  background: var(--panel); border: 1px solid var(--line);
  border-left: 3px solid var(--warn); border-radius: 6px;
  padding: 14px 16px; margin: 22px 0; font-size: 15px;
}
.note strong { color: var(--warn); }
.group { margin: 28px 0; }
.group ul { list-style: none; padding: 0; margin: 0; }
.group li { border-bottom: 1px solid var(--line); }
.group li:last-child { border-bottom: 0; }
.group a { display: block; padding: 9px 2px; text-decoration: none; }
.group a:hover { background: var(--panel); }
footer.site {
  border-top: 1px solid var(--line); margin-top: 48px; padding-top: 18px;
  color: var(--dim); font-size: 13px;
}
.back { display: inline-block; margin-bottom: 20px; font-size: 14px; }
.crumbs { font-size: 14px; margin-bottom: 20px; color: var(--dim); }
.crumbs a { text-decoration: none; }
@media (max-width: 620px) { .wrap { padding: 0 16px 56px; } h1 { font-size: 23px; } }

/* --- the ladder --- */
.stats { display: flex; flex-wrap: wrap; gap: 10px; margin: 18px 0 8px; }
.stat { background: var(--panel); border: 1px solid var(--line); border-radius: 8px;
        padding: 10px 14px; min-width: 120px; }
.stat b { display: block; font-size: 20px; }
.stat span { color: var(--dim); font-size: 12.5px; }
.stage { border: 1px solid var(--line); border-radius: 10px; margin: 18px 0;
         overflow: hidden; }
.stage > header {
  display: flex; align-items: baseline; gap: 10px; flex-wrap: wrap;
  padding: 14px 16px; background: var(--panel); border-bottom: 1px solid var(--line);
}
.stage .n { color: var(--dim); font-size: 12px; letter-spacing: .1em;
            text-transform: uppercase; }
.stage h2 { margin: 0; font-size: 18px; }
.stage h2 a { color: var(--ink); text-decoration: none; }
.stage h2 a:hover { color: var(--accent); }
.price { margin-left: auto; font-size: 12.5px; padding: 2px 10px;
         border-radius: 999px; border: 1px solid var(--line); color: var(--dim); }
.price.free { color: var(--good); border-color: #2c5a3c; }
.price.paid { color: var(--warn); border-color: #5a4a22; }
.stage .blurb { padding: 12px 16px 0; color: var(--dim); font-size: 14.5px; margin: 0; }
.stage .facts { padding: 4px 16px 0; color: var(--dim); font-size: 12.5px;
                letter-spacing: .03em; }
.stage ol { margin: 8px 0 14px; padding: 0 16px 0 40px; }
.stage li { margin: 5px 0; }
.mins { color: var(--dim); font-size: 12.5px; margin-left: 6px; white-space: nowrap; }
.stage li.proj { list-style: none; margin-left: -22px; margin-top: 10px; }
.stage li.proj::before { content: "\\25C6  "; color: var(--warn); }
.obj { color: var(--dim); font-size: 13px; display: block; }

/* --- a lesson --- */
.lesson-head { color: var(--dim); font-size: 13px; letter-spacing: .04em;
               text-transform: uppercase; margin-bottom: 6px; }
.objective { font-size: 17px; color: var(--ink); margin: 0 0 18px; }
.objective b { color: var(--dim); font-weight: 500; }
.check {
  background: var(--panel); border: 1px solid var(--line);
  border-left: 3px solid #3c7a52; border-radius: 6px; padding: 12px 15px;
  margin: 22px 0; font-size: 15px;
}
.check strong { color: var(--good); }
ol.steps { counter-reset: step; list-style: none; padding: 0; margin: 0; }
ol.steps > li { counter-increment: step; position: relative; padding-left: 40px;
                margin: 0 0 18px; }
ol.steps > li::before {
  content: counter(step); position: absolute; left: 0; top: 0;
  width: 26px; height: 26px; border-radius: 50%; text-align: center;
  font-size: 13px; line-height: 26px; background: var(--panel);
  border: 1px solid var(--line); color: var(--accent);
}
ol.steps p { margin: 0 0 8px; }
ol.steps pre { margin: 6px 0 8px; }
.expect { color: var(--dim); font-size: 14.5px; }
.expect b { color: var(--good); font-weight: 600; }
.pitfall { border: 1px solid var(--line); border-radius: 8px; margin: 12px 0;
           padding: 12px 15px; }
.pitfall p { margin: 4px 0; font-size: 15px; }
.pitfall .sym { color: var(--bad); font-weight: 600; }
.pitfall b { color: var(--dim); font-weight: 600; }
dl.terms { margin: 10px 0; }
dl.terms dt { font-weight: 650; margin-top: 10px; }
dl.terms dd { margin: 2px 0 0 0; color: var(--dim); font-size: 15px; }
details { border: 1px solid var(--line); border-radius: 8px; margin: 10px 0;
          padding: 0 14px; background: var(--panel); }
details > summary { cursor: pointer; padding: 11px 0; font-weight: 550; }
details[open] > summary { border-bottom: 1px solid var(--line); margin-bottom: 8px; }
details > div { padding-bottom: 10px; }
.stretch { border: 1px dashed var(--line); border-radius: 8px; padding: 12px 16px; }
.pager { display: flex; justify-content: space-between; gap: 12px;
         margin-top: 38px; font-size: 14px; border-top: 1px solid var(--line);
         padding-top: 16px; }
.pager span { color: var(--dim); }
ul.checklist { list-style: none; padding-left: 0; }
ul.checklist li { padding-left: 28px; position: relative; margin: 8px 0; }
ul.checklist li::before { content: "\\2610"; position: absolute; left: 2px;
                          color: var(--good); }
table.syllabus td.m { white-space: nowrap; color: var(--dim); }

/* --- topics --- */
ul.tags { list-style: none; display: flex; flex-wrap: wrap; gap: 6px;
          padding: 0; margin: 8px 0 0; }
ul.tags li, .stage ul.tags li { margin: 0; }
.tag { display: inline-block; font-size: 12.5px; line-height: 1.3;
       padding: 3px 10px; border: 1px solid var(--line); border-radius: 999px;
       background: var(--panel); color: var(--accent); text-decoration: none; }
.tag:hover { border-color: var(--accent); }
.tag.on { background: var(--accent); border-color: var(--accent); color: var(--bg); }
.tag .count { color: var(--dim); background: var(--line); border-radius: 999px;
              font-size: 11px; padding: 0 6px; margin-left: 6px; }
.objective + ul.tags { margin: -8px 0 18px; }
.builds { color: var(--dim); font-size: 14.5px; margin: 0 0 18px; }
ul.picks { list-style: none; padding: 0; margin: 0; }
ul.picks > li { border-bottom: 1px solid var(--line); padding: 12px 2px; }
ul.picks > li:last-child { border-bottom: 0; }
ul.picks .title { font-weight: 600; }
.why { color: var(--dim); font-size: 13.5px; display: block; margin-top: 2px; }
.quote { display: block; margin-top: 6px; padding-left: 12px; font-size: 14.5px;
         border-left: 3px solid var(--line); color: var(--ink); }
h2.stage-head { font-size: 17px; }
h2.stage-head a { color: var(--ink); text-decoration: none; }
h2.stage-head a:hover { color: var(--accent); }
h2.stage-head .n { color: var(--dim); font-size: 12px; letter-spacing: .1em;
                   text-transform: uppercase; margin-right: 8px; }
"""

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="robots" content="noindex, nofollow">
<title>{title} — 3VN Robotics</title>
<style>{style}</style>
</head>
<body>
<header class="site"><div class="wrap">
  <div class="brand"><a href="{home}">3VN Robotics</a><span class="badge">preview</span></div>
  <div class="meta">{meta}</div>
</div></header>
<div class="wrap">
{body}
<footer class="site">
  Generated from the repository's own course and documentation. {stamp}
</footer>
</div>
</body>
</html>
"""


# --------------------------------------------------------------- utilities

def run(cmd, root):
    """Return trimmed stdout for `cmd`, or '' if it fails."""
    try:
        return subprocess.run(cmd, cwd=root, capture_output=True,
                              text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def render(text):
    """
    Render markdown with the extensions these docs actually use.

    Imported here rather than at module level, so `--check` runs on a
    machine with only PyYAML -- validation should not need the renderer.
    """
    import markdown
    return markdown.markdown(
        text, extensions=['tables', 'fenced_code', 'toc', 'sane_lists'])


def inline(text):
    """Render one short markdown fragment without its wrapping <p>."""
    out = render(str(text).strip())
    if out.startswith('<p>') and out.endswith('</p>') and out.count('<p>') == 1:
        out = out[3:-4]
    return rewrite_links(out)


def esc(text):
    """Escape plain text for HTML."""
    return htmllib.escape(str(text), quote=True)


def rewrite_links(html):
    """
    Point .md links at the generated .html, and keep them relative.

    The docs cross-reference each other as `[x](viewing.md)`, which is
    right for reading them in the repository and wrong once rendered.
    """
    html = re.sub(r'href="([^"#]+?)\.md(#[^"]*)?"', r'href="\1.html\2"', html)
    html = html.replace('href="decisions/', 'href="decisions-')
    # Links up into the repo (src/, scripts/, cad/) have no page here.
    html = re.sub(r'<a href="(?:\.\./)?(?:src|scripts|cad|firmware|\.github)/[^"]*">'
                  r'(.*?)</a>', r'<code>\1</code>', html)
    # Nor does anything else outside docs/ -- THIRD_PARTY.md, deploy/.
    # Keep the text, drop the link: a dead link is worse than none.
    return re.sub(r'<a href="\.\./[^"]*">(.*?)</a>', r'\1', html)


def rebase(html, prefix):
    """
    Re-root a page's relative links for a page `prefix` deep, e.g. '../../'.

    Pages are written as if they all sat at the site root, so every helper
    can say `lesson-x.html`. The topic pages sit two directories down;
    this is what keeps that true for them without a second set of helpers.
    Absolute URLs, scheme links and in-page anchors are left alone.
    """
    if not prefix:
        return html
    return re.sub(r'(\shref=")(?![a-z][a-z0-9+.-]*:|/|#)', r'\g<1>' + prefix, html)


def doc_page(name):
    """Map a docs-relative path to the page it renders as."""
    path = pathlib.PurePosixPath(name)
    if path.parent.name == 'decisions':
        return f'decisions-{path.stem}.html'
    return f'{path.stem}.html'


def doc_label(name):
    """The human title for a doc, from the curriculum where it has one."""
    for _, items in CURRICULUM:
        for doc, label in items:
            if doc == name:
                return label
    stem = pathlib.PurePosixPath(name).stem
    if stem[:4].isdigit():
        return 'ADR-' + stem[:4] + ': ' + stem[5:].replace('-', ' ')
    return stem.replace('-', ' ').capitalize()


def slug(text):
    """A stable anchor for a glossary term."""
    return re.sub(r'[^a-z0-9]+', '-', str(text).lower()).strip('-')


def hours(minutes):
    """Say a duration the way a person would plan around it."""
    if minutes < 90:
        return f'{minutes} min'
    halves = round(minutes / 30)
    whole, half = divmod(halves, 2)
    return f'about {whole}{"½" if half else ""} hours'


# ---------------------------------------------------------------- loading

def load_course(root):
    """
    Load the curriculum from course/*.yaml, in file order.

    The curriculum is DATA, not markup in this script. Adding a lesson is
    adding a block to a YAML file; nothing here needs to know about it.
    That is the same reason the robot's dimensions live in YAML rather
    than in the xacro.
    """
    course = root / 'course'
    if not course.is_dir():
        return []
    stages = [yaml.safe_load(f.read_text()) for f in sorted(course.glob('*.yaml'))
              if f.name != TAGS_FILE]
    number = 0
    for stage in stages:
        lessons = stage.get('lessons') or []
        for position, lesson in enumerate(lessons, 1):
            number += 1
            lesson['index'] = number
            lesson['position'] = position
            lesson['stage'] = stage
        if isinstance(stage.get('project'), dict):
            stage['project']['stage'] = stage
        stage['minutes'] = (
            sum(int(le.get('minutes') or 0) for le in lessons)
            + int((stage.get('project') or {}).get('minutes') or 0))
    return stages


def load_tags(root):
    """
    Load the topic vocabulary from course/tags.yaml, in file order.

    Returns None when the file is missing, which validation reports: a
    lesson's tags mean nothing without the list they are checked against.
    """
    path = root / 'course' / TAGS_FILE
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        return []
    return data.get('tags') or []


def make_targets(root):
    """Every target the Makefile declares."""
    makefile = root / 'Makefile'
    if not makefile.exists():
        return None
    return set(re.findall(r'^([A-Za-z0-9_.-]+)\s*:(?!=)', makefile.read_text(), re.M))


def commands_in(text):
    """Yield every `make <target>` invoked in a block of shell."""
    for line in str(text).splitlines():
        line = line.split('#', 1)[0]
        for match in re.finditer(r'(?:^|[;&|(]\s*|\s)make\s+([A-Za-z0-9_.-]+)', line):
            if '=' not in match.group(1):
                yield match.group(1)


def code_in(markdown_text):
    """
    Return only the code in a block of markdown: fenced blocks and `spans`.

    Prose is not shell. "That would make the arm heavier" mentions make;
    it does not run it.
    """
    text = str(markdown_text)
    blocks = re.findall(r'```[^\n]*\n(.*?)```', text, flags=re.S)
    spans = re.findall(r'`([^`\n]+)`', re.sub(r'```.*?```', '', text, flags=re.S))
    return '\n'.join(blocks + spans)


def doc_links(text):
    """Yield every relative .md link in a block of markdown."""
    for target in re.findall(r'\]\(([^)#\s]+\.md)(?:#[^)]*)?\)', str(text)):
        if '://' not in target:
            yield target


def plain(text):
    """
    Markdown reduced to the words a reader sees, whitespace collapsed.

    Good enough to find a quoted passage: drops emphasis and code marks
    and keeps a link's text. Needs no renderer, so `--check` can use it.
    """
    text = re.sub(r'\[([^\]]*)\]\([^)]*\)', r'\1', str(text))
    text = re.sub(r'\*+|`+', '', text)
    return ' '.join(text.split())


def lesson_text(lesson):
    """Everything a lesson page shows in prose, as plain text."""
    parts = [lesson.get('objective'), lesson.get('body'), lesson.get('checkpoint')]
    for step in lesson.get('steps') or []:
        if isinstance(step, dict):
            parts += [step.get('do'), step.get('expect')]
    for pit in lesson.get('pitfalls') or []:
        parts += [pit.get('symptom'), pit.get('cause'), pit.get('fix')]
    for concept in lesson.get('concepts') or []:
        parts += [concept.get('term'), concept.get('meaning')]
    for item in lesson.get('quiz') or []:
        parts += [item.get('q'), item.get('a')]
    exercise = lesson.get('exercise')
    if isinstance(exercise, dict):
        parts += [exercise.get('task'), exercise.get('hint'), exercise.get('answer')]
    return plain(' '.join(str(part) for part in parts if part))


# ------------------------------------------------------------- validation

def validate(stages, root, tags=None):
    """
    Return every problem with the curriculum, as a list of strings.

    Empty means valid. Checks the shape of each stage, lesson and project,
    and -- more usefully -- that what a lesson points at exists: Make
    targets, docs, topic tags, the lessons it builds on, and the
    uniqueness of ids and glossary terms.
    """
    errors = []
    docs = root / 'docs'
    targets = make_targets(root)
    seen_ids, seen_terms = {}, {}
    order = {le.get('id'): le.get('index')
             for st in stages for le in (st.get('lessons') or [])}

    def need(where, data, fields):
        for name in fields:
            value = data.get(name)
            if value is None or value == '' or value == []:
                errors.append(f'{where}: missing `{name}`')

    def check_doc(where, name):
        if not (docs / name).exists():
            errors.append(f'{where}: links to docs/{name}, which does not exist')

    def check_shell(where, text):
        if targets is None:
            return
        for target in commands_in(text):
            if target not in targets:
                errors.append(f'{where}: runs `make {target}`, '
                              f'which the Makefile does not define')

    def check_markdown(where, text):
        for target in doc_links(text):
            name = target[3:] if target.startswith('../') else target
            if name.startswith('docs/'):
                name = name[5:]
            check_doc(where, name)

    if not stages:
        return ['course/: no stages found']

    known, used = {}, set()
    if tags is None:
        errors.append(f'course/{TAGS_FILE}: missing -- every lesson needs tags from it')
    for i, tag in enumerate(tags or [], 1):
        if not isinstance(tag, dict):
            errors.append(f'course/{TAGS_FILE}: entry {i} is not a slug/label/description')
            continue
        twhere = f'tag {tag.get("slug", i)}'
        need(twhere, tag, TAG_FIELDS)
        name = str(tag.get('slug', ''))
        if name and not re.fullmatch(r'[a-z0-9]+(?:-[a-z0-9]+)*', name):
            errors.append(f'{twhere}: a slug is lower-case letters, digits and hyphens '
                          f'(it is part of a URL)')
        if name in known:
            errors.append(f'{twhere}: listed twice in course/{TAGS_FILE}')
        known[name] = tag

    for expected, stage in enumerate(stages, 1):
        where = f'stage {stage.get("id", expected)}'
        need(where, stage, STAGE_FIELDS)
        if stage.get('number') != expected:
            errors.append(f'{where}: numbered {stage.get("number")}, '
                          f'but it is stage {expected} in file order')
        price = str(stage.get('price', ''))
        if price != 'free' and not re.fullmatch(r'\$\d+', price):
            errors.append(f'{where}: price {price!r} is neither "free" nor "$N"')
        for i, outcome in enumerate(stage.get('outcomes') or []):
            if not isinstance(outcome, str):
                errors.append(f'{where}: outcome {i + 1} is not a sentence')

        for lesson in stage.get('lessons') or []:
            lid = lesson.get('id', '?')
            lwhere = f'lesson {lid}'
            need(lwhere, lesson, LESSON_FIELDS)
            if lid in seen_ids:
                errors.append(f'{lwhere}: id also used in {seen_ids[lid]}')
            seen_ids[lid] = stage.get('id')
            if not isinstance(lesson.get('minutes'), int) or lesson['minutes'] <= 0:
                errors.append(f'{lwhere}: `minutes` must be a positive integer')

            lesson_tags = lesson.get('tags') or []
            if not isinstance(lesson_tags, list):
                errors.append(f'{lwhere}: `tags` must be a list')
                lesson_tags = []
            for tag in lesson_tags:
                if tags is not None and tag not in known:
                    errors.append(f'{lwhere}: tag {tag!r} is not in course/{TAGS_FILE} '
                                  f'(known: {", ".join(known)})')
            if len(set(map(str, lesson_tags))) != len(lesson_tags):
                errors.append(f'{lwhere}: lists the same tag twice')
            used.update(map(str, lesson_tags))

            builds_on = lesson.get('builds_on') or []
            if not isinstance(builds_on, list):
                errors.append(f'{lwhere}: `builds_on` must be a list of lesson ids')
                builds_on = []
            for dep in builds_on:
                if dep not in order:
                    errors.append(f'{lwhere}: builds on {dep!r}, which is not a lesson')
                elif order[dep] >= lesson.get('index', 0):
                    errors.append(f'{lwhere}: builds on {dep!r}, which does not come '
                                  f'before it in the syllabus')

            mentions = lesson.get('mentions') or []
            if not isinstance(mentions, list):
                errors.append(f'{lwhere}: `mentions` must be a list of tag and quote')
                mentions = []
            text = lesson_text(lesson) if mentions else ''
            for i, mention in enumerate(mentions, 1):
                mwhere = f'{lwhere} mention {i}'
                if not isinstance(mention, dict) or not mention.get('tag') \
                        or not mention.get('quote'):
                    errors.append(f'{mwhere}: needs both `tag` and `quote`')
                    continue
                tag, quote = mention['tag'], plain(mention['quote'])
                if tags is not None and tag not in known:
                    errors.append(f'{mwhere}: tag {tag!r} is not in course/{TAGS_FILE}')
                if tag in lesson_tags:
                    errors.append(f'{mwhere}: the lesson is already tagged {tag!r}')
                if len(quote.split()) < 4:
                    errors.append(f'{mwhere}: quote at least four words, so the link '
                                  f'finds the right passage')
                elif quote not in text:
                    errors.append(f'{mwhere}: quote {quote!r} is not in the lesson')

            for i, step in enumerate(lesson.get('steps') or [], 1):
                if not isinstance(step, dict) or not step.get('do'):
                    errors.append(f'{lwhere}: step {i} has no `do`')
                    continue
                check_shell(f'{lwhere} step {i}', step.get('run', ''))
                check_shell(f'{lwhere} step {i}', code_in(step.get('do', '')))
                check_markdown(f'{lwhere} step {i}', step.get('do', ''))

            for field in ('body', 'checkpoint'):
                check_markdown(lwhere, lesson.get(field, ''))
                check_shell(lwhere, code_in(lesson.get(field, '')))

            for name in lesson.get('read') or []:
                check_doc(lwhere, name)

            for i, pit in enumerate(lesson.get('pitfalls') or [], 1):
                for part in ('symptom', 'cause', 'fix'):
                    if not pit.get(part):
                        errors.append(f'{lwhere}: pitfall {i} has no `{part}`')
                check_shell(f'{lwhere} pitfall {i}', code_in(pit.get('fix', '')))

            for i, item in enumerate(lesson.get('quiz') or [], 1):
                if not item.get('q') or not item.get('a'):
                    errors.append(f'{lwhere}: quiz {i} needs both `q` and `a`')

            exercise = lesson.get('exercise')
            if exercise is not None and not (isinstance(exercise, dict)
                                             and exercise.get('task')):
                errors.append(f'{lwhere}: `exercise` needs a `task`')

            for concept in lesson.get('concepts') or []:
                term = str(concept.get('term', '')).strip()
                if not term or not concept.get('meaning'):
                    errors.append(f'{lwhere}: a concept needs `term` and `meaning`')
                    continue
                key = term.lower()
                if key in seen_terms:
                    errors.append(f'{lwhere}: glossary term {term!r} is already '
                                  f'defined by lesson {seen_terms[key]}')
                seen_terms[key] = lid

        project = stage.get('project')
        if isinstance(project, dict):
            pwhere = f'{where} project'
            need(pwhere, project, PROJECT_FIELDS)
            for i, task in enumerate(project.get('tasks') or [], 1):
                text = task.get('do') if isinstance(task, dict) else task
                if not text:
                    errors.append(f'{pwhere}: task {i} is empty')
                if isinstance(task, dict):
                    check_shell(f'{pwhere} task {i}', task.get('run', ''))
                check_shell(f'{pwhere} task {i}', code_in(text or ''))
            for name in project.get('read') or []:
                check_doc(pwhere, name)

    for name in known:
        if name not in used:
            errors.append(f'tag {name}: in course/{TAGS_FILE}, but no lesson carries it')
    return errors


# ---------------------------------------------------------------- pieces

def phase_table(docs):
    """Lift the phase table straight out of roadmap.md, or return ''."""
    roadmap = docs / 'roadmap.md'
    if not roadmap.exists():
        return ''
    rows = [line for line in roadmap.read_text().splitlines()
            if re.match(r'^\|\s*\d+\s*\|', line)]
    if not rows:
        return ''
    header = '| Phase | What | Status |\n|---|---|---|\n'
    return header + '\n'.join(rows)


def stage_word(stage):
    """`kind: tier` marks the stages planned as paid; free or not, they keep the name."""
    return 'Tier' if stage.get('kind') == 'tier' else 'Stage'


def price_badge(stage):
    """The price pill shown on every stage header."""
    paid = stage.get('price', 'free') != 'free'
    label = stage['price'] + (' &#128274;' if paid else '')
    return f'<span class="price {"paid" if paid else "free"}">{label}</span>'


def tag_href(name):
    """Where a topic's page lives, from the site root."""
    return f'tags/{name}/index.html'


def tag_chips(names, vocab, current=None, counts=None):
    """
    Render tags as links to their topic pages.

    Always in the vocabulary's order, so a tag sits in the same place on
    every page. `current` marks the topic whose page this is; `counts`
    adds how many lessons carry each tag.
    """
    order = list(vocab)
    items = []
    for name in sorted(names, key=order.index):
        on = name == current
        attrs = ' class="tag on" aria-current="page"' if on else ' class="tag"'
        count = f'<span class="count">{counts[name]}</span>' if counts else ''
        items.append(f'<li><a{attrs} href="{tag_href(name)}">'
                     f'{esc(vocab[name]["label"])}{count}</a></li>')
    return f'<ul class="tags" aria-label="Topics">{"".join(items)}</ul>'


def lesson_link(lesson):
    """A lesson's title as a link."""
    return f'<a href="lesson-{lesson["id"]}.html">{esc(lesson["title"])}</a>'


def passage_href(lesson, quote):
    """
    Link to the passage a quote comes from, as a URL text fragment.

    `#:~:text=` scrolls to and highlights the passage in current Chrome,
    Edge, Safari and Firefox, with no script and no anchor to maintain.
    A browser without it opens the lesson at the top. `-`, `,` and `&`
    mean something inside a text fragment, so they are escaped too.
    """
    words = urllib.parse.quote(plain(quote), safe='').replace('-', '%2D')
    return f'lesson-{lesson["id"]}.html#:~:text={words}'


def mentioning(name, stages):
    """[(lesson, quote)] for every lesson that mentions a topic, in order."""
    return [(le, m['quote']) for st in stages for le in st['lessons']
            for m in le.get('mentions') or [] if m['tag'] == name]


def where(lesson):
    """Where a lesson sits: stage, position, length."""
    stage = lesson['stage']
    return (f'{stage_word(stage)} {stage["number"]} &middot; lesson '
            f'{lesson["position"]} &middot; {lesson["minutes"]} min')


def said(items):
    """Join a short list the way a sentence would: a, b and c."""
    items = list(items)
    return items[0] if len(items) == 1 else ', '.join(items[:-1]) + ' and ' + items[-1]


def recommend(name, stages, picks=RELATED_PICKS):
    """
    What to read alongside a topic's lessons, none of which carry the tag.

    Two lists, in this order, and the topic page says so:

      1. Every lesson the topic's lessons BUILD ON (their `builds_on`),
         in syllabus order -- the gaps a reader jumping straight to a
         topic would otherwise fall into.
      2. Up to `picks` lessons that share at least TWO topics with this
         topic's lessons -- one shared tag is too weak a link, and on a
         small topic it only ever picks the course's biggest tag. A lesson
         scores, for each of its tags, the number of this topic's lessons
         that also carry it; ties go to the lesson sharing more reference
         reading (`read`) with them, then to the earlier one.

    Lessons that only MENTION the topic are listed separately and never
    recommended as well. Returns [(lesson, [lessons that build on it])]
    and [(lesson, [the tags it shares, most shared first])].
    """
    lessons = [le for st in stages for le in st['lessons']]
    tagged = [le for le in lessons if name in le['tags']]
    inside = {le['id'] for le in tagged}
    # Listed under "Also mentioned in" already; never twice on one page.
    inside |= {le['id'] for le, _ in mentioning(name, stages)}

    needed_by = {}
    for le in tagged:
        for dep in le.get('builds_on') or []:
            if dep not in inside:
                needed_by.setdefault(dep, []).append(le)
    first = [(le, needed_by[le['id']]) for le in lessons if le['id'] in needed_by]

    together = {}
    for le in tagged:
        for tag in le['tags']:
            if tag != name:
                together[tag] = together.get(tag, 0) + 1
    reading = {doc for le in tagged for doc in le.get('read') or []}
    scored = []
    for le in lessons:
        if le['id'] in inside or le['id'] in needed_by:
            continue
        shares = [tag for tag in le['tags'] if tag in together]
        if len(shares) >= 2:
            score = sum(together[tag] for tag in shares)
            overlap = len(reading.intersection(le.get('read') or []))
            scored.append((-score, -overlap, le['index'], le))
    scored.sort(key=lambda item: item[:3])
    alongside = [(le, sorted((t for t in le['tags'] if t in together),
                             key=lambda t: -together[t]))
                 for *_, le in scored[:picks]]
    return first, alongside


def code_block(text):
    """Render a shell snippet as a fenced block."""
    return render('```bash\n' + str(text).strip() + '\n```')


def read_list(names):
    """Render a list of docs as links to their rendered pages."""
    items = ''.join(f'<li><a href="{doc_page(n)}">{esc(doc_label(n))}</a></li>'
                    for n in names)
    return f'<ul>{items}</ul>'


def pager(sequence, position):
    """Previous/next links through lessons AND projects, in order."""
    def link(item, arrow_left):
        kind, obj = item
        href = (f'lesson-{obj["id"]}.html' if kind == 'lesson'
                else f'project-{obj["stage"]["id"]}.html')
        title = obj['title'] if kind == 'lesson' else f'Project: {obj["title"]}'
        return (f'<a href="{href}">&larr; {esc(title)}</a>' if arrow_left
                else f'<a href="{href}">{esc(title)} &rarr;</a>')
    prev = link(sequence[position - 1], True) if position else '<span></span>'
    nxt = (link(sequence[position + 1], False) if position + 1 < len(sequence)
           else '<span>end of the course</span>')
    return f'<div class="pager">{prev}{nxt}</div>'


def crumbs(stage):
    """Breadcrumb back to the course and the stage."""
    return (f'<div class="crumbs"><a href="index.html">The course</a> / '
            f'<a href="stage-{stage["id"]}.html">{stage_word(stage)} '
            f'{stage["number"]}: {esc(stage["title"])}</a></div>')


# ------------------------------------------------------------------ pages

def lesson_page(lesson, sequence, position, vocab, by_id):
    """
    Render one lesson.

    Always in the same order, so a student learns where to look: why it
    matters, what to do, how to tell it worked, what usually goes wrong,
    the words it introduced, whether it stuck, and where to go further.
    """
    stage = lesson['stage']
    count = len(stage['lessons'])
    parts = [
        crumbs(stage),
        f'<div class="lesson-head">{stage_word(stage)} {stage["number"]} '
        f'&middot; lesson {lesson["position"]} of {count} &middot; '
        f'{hours(lesson["minutes"])}</div>',
        f'<h1>{esc(lesson["title"])}</h1>',
        f'<p class="objective"><b>You will:</b> {inline(lesson["objective"])}</p>',
        tag_chips(lesson['tags'], vocab),
    ]
    if lesson.get('builds_on'):
        parts.append('<p class="builds">Builds on '
                     + said(lesson_link(by_id[dep]) for dep in lesson['builds_on'])
                     + '.</p>')
    touched = []
    for mention in lesson.get('mentions') or []:
        if mention['tag'] not in touched:
            touched.append(mention['tag'])
    if touched:
        parts.append('<p class="builds">Also touches on '
                     + said(f'<a href="{tag_href(t)}">{esc(vocab[t]["label"])}</a>'
                            for t in touched) + '.</p>')
    parts.append(rewrite_links(render(lesson.get('body', ''))))

    parts.append('<h2>Do it</h2><ol class="steps">')
    for step in lesson['steps']:
        piece = [f'<li>{rewrite_links(render(step["do"]))}']
        if step.get('run'):
            piece.append(code_block(step['run']))
        if step.get('expect'):
            piece.append(f'<p class="expect"><b>Expect:</b> {inline(step["expect"])}</p>')
        piece.append('</li>')
        parts.append(''.join(piece))
    parts.append('</ol>')

    parts.append(f'<div class="check"><strong>You will know it worked '
                 f'when:</strong> {inline(lesson["checkpoint"])}</div>')

    if lesson.get('pitfalls'):
        parts.append('<h2>When it goes wrong</h2>')
        for pit in lesson['pitfalls']:
            parts.append(
                f'<div class="pitfall"><p class="sym">{inline(pit["symptom"])}</p>'
                f'<p><b>Why:</b> {inline(pit["cause"])}</p>'
                f'<p><b>Fix:</b> {inline(pit["fix"])}</p></div>')

    if lesson.get('concepts'):
        parts.append('<h2>Terms</h2><dl class="terms">')
        for concept in lesson['concepts']:
            parts.append(f'<dt id="{slug(concept["term"])}">{esc(concept["term"])}</dt>'
                         f'<dd>{inline(concept["meaning"])}</dd>')
        parts.append('</dl><p class="expect">Every term in the course: '
                     '<a href="glossary.html">glossary</a>.</p>')

    if lesson.get('quiz'):
        parts.append('<h2>Check yourself</h2>')
        parts.append('<p class="expect">Answer in your head first, then open it.</p>')
        for item in lesson['quiz']:
            parts.append(f'<details><summary>{inline(item["q"])}</summary>'
                         f'<div>{rewrite_links(render(item["a"]))}</div></details>')

    exercise = lesson.get('exercise')
    if exercise:
        parts.append('<h2>Stretch</h2><div class="stretch">')
        parts.append(rewrite_links(render(exercise['task'])))
        if exercise.get('hint'):
            parts.append(f'<details><summary>Hint</summary><div>'
                         f'{rewrite_links(render(exercise["hint"]))}</div></details>')
        if exercise.get('answer'):
            parts.append(f'<details><summary>One way to do it</summary><div>'
                         f'{rewrite_links(render(exercise["answer"]))}</div></details>')
        parts.append('</div>')

    if lesson.get('read'):
        parts.append('<h2>Going deeper</h2>' + read_list(lesson['read']))

    parts.append(pager(sequence, position))
    return '\n'.join(parts)


def project_page(stage, sequence, position):
    """Render a stage's closing project: a brief, tasks, and a definition of done."""
    project = stage['project']
    parts = [
        crumbs(stage),
        f'<div class="lesson-head">{stage_word(stage)} {stage["number"]} '
        f'&middot; project &middot; {hours(project["minutes"])}</div>',
        f'<h1>Project: {esc(project["title"])}</h1>',
        rewrite_links(render(project['brief'])),
        '<h2>Tasks</h2><ol class="steps">',
    ]
    for task in project['tasks']:
        if isinstance(task, dict):
            piece = f'<li>{rewrite_links(render(task["do"]))}'
            if task.get('run'):
                piece += code_block(task['run'])
            parts.append(piece + '</li>')
        else:
            parts.append(f'<li>{rewrite_links(render(task))}</li>')
    parts.append('</ol>')

    parts.append('<h2>Done when</h2><ul class="checklist">')
    parts.extend(f'<li>{inline(item)}</li>' for item in project['done_when'])
    parts.append('</ul>')

    for hint in project.get('hints') or []:
        parts.append(f'<details><summary>{inline(hint["title"])}</summary>'
                     f'<div>{rewrite_links(render(hint["body"]))}</div></details>')

    if project.get('read'):
        parts.append('<h2>Going deeper</h2>' + read_list(project['read']))
    parts.append(pager(sequence, position))
    return '\n'.join(parts)


def stage_page(stage, sequence, vocab):
    """Render a stage overview: outcomes, prerequisites, syllabus, project."""
    lessons = stage['lessons']
    parts = [
        '<div class="crumbs"><a href="index.html">The course</a></div>',
        f'<div class="lesson-head">{stage_word(stage)} {stage["number"]} '
        f'&middot; {len(lessons)} lessons and a project &middot; '
        f'{hours(stage["minutes"])}</div>',
        f'<h1>{esc(stage["title"])} {price_badge(stage)}</h1>',
        rewrite_links(render(stage['summary'])),
        '<h2>By the end you will be able to</h2><ul>',
    ]
    parts.extend(f'<li>{inline(o)}</li>' for o in stage['outcomes'])
    parts.append('</ul>')

    if stage.get('prerequisites'):
        parts.append('<h2>Before you start</h2><ul>')
        parts.extend(f'<li>{inline(p)}</li>' for p in stage['prerequisites'])
        parts.append('</ul>')

    parts.append('<h2>Lessons</h2><table class="syllabus"><tr><th>#</th>'
                 '<th>Lesson</th><th>Time</th></tr>')
    for lesson in lessons:
        parts.append(
            f'<tr><td>{lesson["position"]}</td><td><a href="lesson-{lesson["id"]}.html">'
            f'{esc(lesson["title"])}</a><span class="obj">{inline(lesson["objective"])}'
            f'</span>{tag_chips(lesson["tags"], vocab)}</td>'
            f'<td class="m">{lesson["minutes"]} min</td></tr>')
    project = stage['project']
    parts.append(
        f'<tr><td>&#9670;</td><td><a href="project-{stage["id"]}.html">Project: '
        f'{esc(project["title"])}</a></td><td class="m">{project["minutes"]} min</td></tr>')
    parts.append('</table>')

    first = f'lesson-{lessons[0]["id"]}.html'
    parts.append(f'<p><a href="{first}">Start with lesson 1 &rarr;</a></p>')
    return '\n'.join(parts)


def glossary_page(stages):
    """Every term the course introduces, alphabetically, linked to where."""
    entries = []
    for stage in stages:
        for lesson in stage['lessons']:
            for concept in lesson.get('concepts') or []:
                entries.append((concept['term'], concept['meaning'], lesson))
    entries.sort(key=lambda e: e[0].lower())
    parts = ['<div class="crumbs"><a href="index.html">The course</a></div>',
             '<h1>Glossary</h1>',
             render(f'{len(entries)} terms, each linked to the lesson that '
                    'introduces it.'),
             '<dl class="terms">']
    for term, meaning, lesson in entries:
        stage = lesson['stage']
        parts.append(
            f'<dt id="{slug(term)}">{esc(term)}</dt><dd>{inline(meaning)} '
            f'<a href="lesson-{lesson["id"]}.html">{stage_word(stage)} '
            f'{stage["number"]}, lesson {lesson["position"]}</a></dd>')
    parts.append('</dl>')
    return '\n'.join(parts)


def tag_page(tag, stages, vocab):
    """
    One topic: every lesson that carries it, in syllabus order and grouped
    by stage, then what to read alongside them.
    """
    name = tag['slug']
    tagged = [le for st in stages for le in st['lessons'] if name in le['tags']]
    groups = [(st, [le for le in st['lessons'] if name in le['tags']]) for st in stages]
    groups = [(st, rows) for st, rows in groups if rows]
    minutes = sum(le['minutes'] for le in tagged)
    lessons_word = 'lesson' if len(tagged) == 1 else 'lessons'
    stages_word = 'stage' if len(groups) == 1 else 'stages'
    parts = [
        '<div class="crumbs"><a href="index.html">The course</a> / '
        '<a href="tags/index.html">Topics</a></div>',
        f'<div class="lesson-head">Topic &middot; {len(tagged)} {lessons_word} in '
        f'{len(groups)} {stages_word} &middot; {hours(minutes)}</div>',
        f'<h1>{esc(tag["label"])}</h1>',
        f'<p class="objective">{inline(tag["description"])}</p>',
    ]
    for stage, rows in groups:
        parts.append(
            f'<h2 class="stage-head"><span class="n">{stage_word(stage)} '
            f'{stage["number"]}</span><a href="stage-{stage["id"]}.html">'
            f'{esc(stage["title"])}</a></h2><ul class="picks">')
        for le in rows:
            parts.append(
                f'<li><span class="title">{lesson_link(le)}</span>'
                f'<span class="mins">lesson {le["position"]} &middot; {le["minutes"]} min</span>'
                f'<span class="obj">{inline(le["objective"])}</span>'
                f'{tag_chips(le["tags"], vocab, current=name)}</li>')
        parts.append('</ul>')

    mentions = mentioning(name, stages)
    if mentions:
        parts.append('<h2>Also mentioned in</h2>')
        parts.append(render(
            f'Lessons about something else that touch on {tag["label"]} in one '
            'passage. Each link opens the lesson at that passage.'))
        parts.append('<ul class="picks">')
        for le, quote in mentions:
            parts.append(
                f'<li><span class="title"><a href="{passage_href(le, quote)}">'
                f'{esc(le["title"])}</a></span><span class="why">{where(le)}</span>'
                f'<span class="quote">&ldquo;{esc(plain(quote))}&rdquo;</span></li>')
        parts.append('</ul>')

    first, alongside = recommend(name, stages)
    if first or alongside:
        parts.append('<h2>Recommended</h2>')
        parts.append(render(
            f'Not tagged {tag["label"]}, but worth reading with it: first the '
            'lessons the ones above **build on**, in syllabus order, then the '
            'lessons whose topics **most often appear alongside** this one.'))
        parts.append('<ul class="picks">')
        for le, needers in first:
            parts.append(
                f'<li><span class="title">{lesson_link(le)}</span>'
                f'<span class="why">{where(le)} &middot; '
                f'{said(esc(n["title"]) for n in needers)} '
                f'build{"s" if len(needers) == 1 else ""} on it</span></li>')
        for le, shared in alongside:
            parts.append(
                f'<li><span class="title">{lesson_link(le)}</span>'
                f'<span class="why">{where(le)} &middot; shares '
                f'{said(esc(vocab[t]["label"]) for t in shared)}</span></li>')
        parts.append('</ul>')

    parts.append('<h2>Every topic</h2>')
    parts.append(tag_chips(list(vocab), vocab, current=name))
    return '\n'.join(parts)


def tags_index(stages, vocab):
    """Every topic, with how many lessons carry it."""
    counts = {name: 0 for name in vocab}
    for st in stages:
        for le in st['lessons']:
            for tag in le['tags']:
                counts[tag] += 1
    parts = ['<div class="crumbs"><a href="index.html">The course</a></div>',
             '<h1>Topics</h1>',
             render(f'Every lesson carries one or more of these {len(vocab)} '
                    'topics. Each topic page lists every lesson that carries it, '
                    'across the whole course, and what to read alongside them.'),
             '<ul class="picks">']
    for name, tag in vocab.items():
        word = 'lesson' if counts[name] == 1 else 'lessons'
        parts.append(
            f'<li><span class="title"><a href="{tag_href(name)}">{esc(tag["label"])}</a>'
            f'</span><span class="mins">{counts[name]} {word}</span>'
            f'<span class="obj">{inline(tag["description"])}</span></li>')
    parts.append('</ul>')
    return '\n'.join(parts)


def ladder_html(stages):
    """Render the stage/tier ladder: what you get, and what it costs."""
    out = []
    for stage in stages:
        items = ''.join(
            f'<li><a href="lesson-{le["id"]}.html">{esc(le["title"])}</a>'
            f'<span class="mins">{le["minutes"]} min</span></li>'
            for le in stage['lessons'])
        project = stage['project']
        items += (f'<li class="proj"><a href="project-{stage["id"]}.html">Project: '
                  f'{esc(project["title"])}</a></li>')
        out.append(
            f'<section class="stage"><header>'
            f'<span class="n">{stage_word(stage)} {stage["number"]}</span>'
            f'<h2><a href="stage-{stage["id"]}.html">{esc(stage["title"])}</a></h2>'
            f'{price_badge(stage)}</header>'
            f'<p class="blurb">{inline(stage["summary"])}</p>'
            f'<div class="facts">{len(stage["lessons"])} lessons &middot; '
            f'project &middot; {hours(stage["minutes"])}</div>'
            f'<ol>{items}</ol></section>')
    return '\n'.join(out)


def index_html(docs, written, stages, vocab):
    """Render the front page: the course first, the manual second."""
    lessons = [le for st in stages for le in st['lessons']]
    total_minutes = sum(st['minutes'] for st in stages)
    listed = {name for _, items in CURRICULUM for name, _ in items}
    free = [st for st in stages if st.get('price') == 'free']
    paid = [st for st in stages if st.get('price') != 'free']
    cost = (f"${sum(int(st['price'].strip('$')) for st in paid)}" if paid else '$0')

    parts = ['<h1>Build a robot the way you build software</h1>', render(
        'One system that grows, from a simulated arm to a mobile '
        'manipulator that sees and learns. You already know Git, Docker, '
        'testing, CI and observability. This is those skills, pointed at '
        'a machine that moves.')]

    if paid:
        price_stat = (f'<div class="stat"><b>{len(free)} free</b><span>then '
                      f'{paid[0]["price"]} per tier, {cost} for all</span></div>')
    else:
        price_stat = (f'<div class="stat"><b>Free</b><span>all {len(stages)} '
                      'stages, for now</span></div>')

    parts.append(
        '<div class="stats">'
        f'<div class="stat"><b>{len(lessons)}</b><span>lessons</span></div>'
        f'<div class="stat"><b>{len(stages)}</b>'
        '<span>stages, each ending in a project</span></div>'
        f'<div class="stat"><b>~{round(total_minutes / 60)} h</b>'
        '<span>of hands-on work</span></div>'
        f'{price_stat}'
        '</div>')

    parts.append(
        '<div class="note"><strong>This is a live preview of an '
        'unfinished course.</strong> It is generated from the project as '
        'it actually stands, including what does not work yet. Where '
        'something is unproven or failing, the lesson says so.</div>')

    parts.append('<h2>How every lesson works</h2>')
    parts.append(render(
        'Each lesson has the same shape, so you always know where to look. '
        '**Why** the thing is built this way, usually through something '
        'that went wrong here first. **Do it** — numbered steps with the '
        'exact command and what you should see. **A checkpoint** that tells '
        'you it worked. **When it goes wrong** — the symptoms that mislead, '
        'and what they really mean. **Check yourself** — questions with the '
        'answers folded away. **Stretch** — an exercise that goes past the '
        'steps. Every stage ends in a **project** with a definition of done.\n\n'
        'You need Docker and a machine with 6 GB of memory to give it. You '
        'do not need a robot: hardware is one of three interchangeable '
        'targets, and the course is built so the other two teach '
        'everything except the screwdriver.'))

    parts.append('<h2>The course</h2>')
    parts.append(ladder_html(stages))
    parts.append('<p><a href="glossary.html">Glossary of every term the course '
                 'introduces &rarr;</a></p>')

    counts = {name: 0 for name in vocab}
    for le in lessons:
        for tag in le['tags']:
            counts[tag] += 1
    parts.append('<h2>Topics</h2>')
    parts.append(render(
        'Every lesson is tagged. Pick a topic to see each lesson that covers '
        'it, wherever it sits in the course, and what to read alongside.'))
    parts.append(tag_chips(list(vocab), vocab, counts=counts))
    parts.append('<p><a href="tags/index.html">Every topic, described &rarr;</a></p>')

    table = phase_table(docs)
    if table:
        parts.append('<h2>The real project, honestly</h2>')
        parts.append(render(
            'The course is written against a project that is genuinely '
            'being built, not a finished example. This is where it is:'))
        parts.append(render(table))

    parts.append('<h2>Reference</h2>')
    parts.append(render(
        'The project\'s own documentation, linked from the lessons that '
        'need it. You do not have to read it in order.'))
    for heading, items in CURRICULUM:
        rows = [f'<li><a href="{doc_page(n)}">{esc(label)}</a></li>'
                for n, label in items if n in written]
        if rows:
            parts.append(f'<div class="group"><h3>{heading}</h3><ul>'
                         + ''.join(rows) + '</ul></div>')

    extra = sorted(set(written) - listed)
    if extra:
        rows = ''.join(f'<li><a href="{doc_page(n)}">{esc(doc_label(n))}</a></li>'
                       for n in extra)
        parts.append('<div class="group"><h3>Everything else</h3><ul>'
                     + rows + '</ul></div>')

    adrs = sorted((docs / 'decisions').glob('*.md'))
    if adrs:
        rows = ''.join(
            f'<li><a href="decisions-{a.stem}.html">'
            f'{esc(doc_label("decisions/" + a.name))}</a></li>' for a in adrs)
        parts.append('<div class="group"><h3>Decisions</h3><ul>'
                     + rows + '</ul></div>')
    return '\n'.join(parts)


# ------------------------------------------------------------------ build

def build(root, out, stages, tags):
    """Render the whole site into `out`. Returns the page count."""
    docs = root / 'docs'
    out.mkdir(parents=True, exist_ok=True)

    version = (root / 'VERSION').read_text().strip() if (root / 'VERSION').exists() else ''
    # git is absent inside the build image, so the commit is passed in.
    commit = (run(['git', 'rev-parse', '--short', 'HEAD'], root)
              or os.environ.get('THREEVN_GIT_COMMIT', '')[:7])
    stamp = f'v{version} · {commit}'
    meta = f'v{version} · {commit or "no commit"} · built {datetime.date.today().isoformat()}'

    def write(name, title, body):
        # `name` may be a path: the topic pages sit in tags/<slug>/.
        prefix = '../' * name.count('/')
        path = out / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(PAGE.format(
            title=esc(title), style=STYLE, meta=meta, stamp=stamp,
            home=prefix + 'index.html', body=rebase(body, prefix)))

    vocab = {tag['slug']: tag for tag in tags}
    by_id = {le['id']: le for st in stages for le in st['lessons']}

    written = []
    for source in sorted(docs.glob('*.md')):
        html = rewrite_links(render(source.read_text()))
        write(source.stem + '.html', doc_label(source.name),
              f'<a class="back" href="index.html">&larr; the course</a>\n{html}')
        written.append(source.name)

    for source in sorted((docs / 'decisions').glob('*.md')):
        html = rewrite_links(render(source.read_text()))
        write(f'decisions-{source.stem}.html', doc_label('decisions/' + source.name),
              f'<a class="back" href="index.html">&larr; the course</a>\n{html}')

    sequence = []
    for stage in stages:
        sequence.extend(('lesson', le) for le in stage['lessons'])
        sequence.append(('project', stage['project']))

    for position, (kind, obj) in enumerate(sequence):
        if kind == 'lesson':
            write(f'lesson-{obj["id"]}.html', obj['title'],
                  lesson_page(obj, sequence, position, vocab, by_id))
        else:
            write(f'project-{obj["stage"]["id"]}.html', f'Project: {obj["title"]}',
                  project_page(obj['stage'], sequence, position))

    for stage in stages:
        write(f'stage-{stage["id"]}.html', stage['title'],
              stage_page(stage, sequence, vocab))

    for tag in tags:
        write(tag_href(tag['slug']), f'Topic: {tag["label"]}', tag_page(tag, stages, vocab))
    write('tags/index.html', 'Topics', tags_index(stages, vocab))

    write('glossary.html', 'Glossary', glossary_page(stages))
    write('index.html', 'The course', index_html(docs, written, stages, vocab))
    (out / 'robots.txt').write_text('User-agent: *\nDisallow: /\n')
    return len(list(out.rglob('*.html')))


def main(argv=None):
    """Validate the curriculum, then build the static course site."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--root', default='.', help='repository root')
    parser.add_argument('--out', default='site', help='output directory')
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('--check', action='store_true',
                        help='validate the curriculum and render nothing')
    args = parser.parse_args(argv)

    root = pathlib.Path(args.root).resolve()
    stages = load_course(root)
    tags = load_tags(root)
    errors = validate(stages, root, tags)
    if errors:
        print(f'  course/ has {len(errors)} problem(s):', file=sys.stderr)
        for error in errors:
            print(f'    {error}', file=sys.stderr)
        return 1

    lessons = sum(len(st['lessons']) for st in stages)
    minutes = sum(st['minutes'] for st in stages)
    print(f'  course ok: {len(stages)} stages, {lessons} lessons, '
          f'{len(stages)} projects, {len(tags)} topics, {hours(minutes)}')
    if args.check:
        return 0

    out = pathlib.Path(args.out).resolve()
    if args.clean and out.exists():
        shutil.rmtree(out)
    count = build(root, out, stages, tags)
    print(f'  {count} pages -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

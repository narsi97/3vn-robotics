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
Render this repository's documentation into a static course site.

    make site

The site IS the docs. Nothing is written for it separately, and that is
the point: the course has to stay true as the project moves, and a
second copy of the explanation would drift from the first within a week.
Phase status comes out of docs/roadmap.md, the version out of VERSION,
the commit out of git. Deepen the project and the site deepens with it.

Three constraints come from where this is served -- behind the shared
3VNSYSTEMS Caddy, on an unlisted path:

  * Every link is RELATIVE. The site lives under /rc-<something>/ today
    and may live at a bare domain tomorrow, and that move is supposed to
    be a config change rather than a rewrite.
  * No JavaScript at all. The enforced CSP is `script-src 'self'` with
    no 'unsafe-inline', so an inline <script> is dead on arrival. Styles
    are inline because `style-src` does allow 'unsafe-inline'.
  * Every page carries noindex, and robots.txt disallows everything.
    Unlisted means unlisted; this is obscurity, not security, exactly as
    projects-for-businesses/README.md says of the client sites.
"""

import argparse
import datetime
import os
import pathlib
import re
import shutil
import subprocess
import sys

import markdown
import yaml

#: The reading order. Anything in docs/ that is not named here still gets
#: rendered, under Reference -- so adding a doc never means it silently
#: fails to appear, which is how a hand-maintained index rots.
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
        ('testing.md', 'Testing'),
        ('cicd.md', 'CI/CD'),
        ('docker.md', 'Docker'),
        ('deployment.md', 'Deployment'),
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
    ]),
]

STYLE = """
:root {
  --bg: #11131a; --panel: #181b24; --line: #272b36;
  --ink: #e6e8ef; --dim: #9aa0b0; --accent: #7fb7ff; --warn: #ffcc66;
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
h2 { font-size: 20px; margin: 34px 0 10px; }
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
table { border-collapse: collapse; width: 100%; margin: 16px 0; font-size: 14.5px; }
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
.group .sub { color: var(--dim); font-size: 13px; }
footer.site {
  border-top: 1px solid var(--line); margin-top: 48px; padding-top: 18px;
  color: var(--dim); font-size: 13px;
}
.back { display: inline-block; margin-bottom: 20px; font-size: 14px; }
@media (max-width: 620px) { .wrap { padding: 0 16px 56px; } h1 { font-size: 23px; } }

/* --- the ladder --- */
.stage { border: 1px solid var(--line); border-radius: 10px; margin: 18px 0;
         overflow: hidden; }
.stage > header {
  display: flex; align-items: baseline; gap: 10px; flex-wrap: wrap;
  padding: 14px 16px; background: var(--panel); border-bottom: 1px solid var(--line);
}
.stage .n { color: var(--dim); font-size: 12px; letter-spacing: .1em;
            text-transform: uppercase; }
.stage h2 { margin: 0; font-size: 18px; }
.price { margin-left: auto; font-size: 12.5px; padding: 2px 10px;
         border-radius: 999px; border: 1px solid var(--line); color: var(--dim); }
.price.free { color: #7ddc9a; border-color: #2c5a3c; }
.price.paid { color: var(--warn); border-color: #5a4a22; }
.stage .blurb { padding: 12px 16px 2px; color: var(--dim); font-size: 14.5px; }
.stage ol { margin: 8px 0 14px; padding: 0 16px 0 40px; }
.stage li { margin: 5px 0; }
.stage li .obj { color: var(--dim); font-size: 13px; display: block; }
.lesson-head { color: var(--dim); font-size: 13px; letter-spacing: .04em;
               text-transform: uppercase; margin-bottom: 6px; }
.objective { font-size: 17px; color: var(--ink); margin: 0 0 18px; }
.check {
  background: var(--panel); border: 1px solid var(--line);
  border-left: 3px solid #3c7a52; border-radius: 6px; padding: 12px 15px;
  margin: 22px 0; font-size: 15px;
}
.check strong { color: #7ddc9a; }
.pager { display: flex; justify-content: space-between; gap: 12px;
         margin-top: 34px; font-size: 14px; }
.pager span { color: var(--dim); }
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
  <div class="brand"><a href="{home}">3VN Robotics</a><span class="badge">unlisted preview</span></div>
  <div class="meta">{meta}</div>
</div></header>
<div class="wrap">
{body}
<footer class="site">
  Generated from the repository's own documentation. {stamp}
</footer>
</div>
</body>
</html>
"""


def run(cmd, root):
    """Return trimmed stdout for `cmd`, or '' if it fails."""
    try:
        return subprocess.run(cmd, cwd=root, capture_output=True,
                              text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


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


def render(text):
    """Render markdown with the extensions these docs actually use."""
    return markdown.markdown(
        text, extensions=['tables', 'fenced_code', 'toc', 'sane_lists'])


def rewrite_links(html):
    """
    Point .md links at the generated .html, and keep them relative.

    The docs cross-reference each other as `[x](viewing.md)`, which is
    right for reading them in the repository and wrong once rendered.
    """
    html = re.sub(r'href="([^"]+?)\.md"', r'href="\1.html"', html)
    html = re.sub(r'href="decisions/([^"]+?)\.html"',
                  r'href="decisions-\1.html"', html)
    # Links up into the repo (src/, scripts/, cad/) have no page here.
    return re.sub(r'<a href="(?:\.\./)?(?:src|scripts|cad|firmware)/[^"]*">'
                  r'(.*?)</a>', r'<code>\1</code>', html)


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
    stages = [yaml.safe_load(f.read_text()) for f in sorted(course.glob('*.yaml'))]
    number = 0
    for stage in stages:
        for lesson in stage.get('lessons', []):
            number += 1
            lesson['index'] = number
            lesson['stage'] = stage
    return stages


def lesson_page(lesson, previous, nxt, total):
    """Render one lesson: objective, what to run, why, and the checkpoint."""
    stage = lesson['stage']
    parts = [
        '<a class="back" href="index.html">&larr; the course</a>',
        f'<div class="lesson-head">Stage {stage["number"]} &middot; '
        f'{stage["title"]} &middot; lesson {lesson["index"]} of {total}</div>',
        f'<h1>{lesson["title"]}</h1>',
        f'<p class="objective">{lesson["objective"]}</p>',
    ]
    if lesson.get('commands'):
        parts.append(render('```bash\n' + lesson['commands'].strip() + '\n```'))
    parts.append(rewrite_links(render(lesson.get('body', ''))))
    if lesson.get('checkpoint'):
        parts.append(f'<div class="check"><strong>You will know it worked '
                     f'when:</strong> {lesson["checkpoint"].strip()}</div>')
    if lesson.get('doc'):
        target = pathlib.Path(lesson['doc']).stem + '.html'
        parts.append(f'<p>Going deeper: <a href="{target}">'
                     f'{lesson["doc"]}</a></p>')
    prev_link = (f'<a href="lesson-{previous["id"]}.html">&larr; '
                 f'{previous["title"]}</a>' if previous else '<span></span>')
    next_link = (f'<a href="lesson-{nxt["id"]}.html">{nxt["title"]} '
                 f'&rarr;</a>' if nxt else '<span>end of the course</span>')
    parts.append(f'<div class="pager">{prev_link}{next_link}</div>')
    return '\n'.join(parts)


def ladder_html(stages):
    """Render the stage/tier ladder: what you get, and what it costs."""
    out = []
    for stage in stages:
        paid = stage.get('price', 'free') != 'free'
        label = stage['price'] + (' &#128274;' if paid else '')
        kind = 'paid' if paid else 'free'
        word = 'Tier' if stage.get('kind') == 'tier' else 'Stage'
        items = ''.join(
            f'<li><a href="lesson-{le["id"]}.html">{le["title"]}</a>'
            f'<span class="obj">{le["objective"]}</span></li>'
            for le in stage.get('lessons', []))
        out.append(
            f'<section class="stage"><header>'
            f'<span class="n">{word} {stage["number"]}</span>'
            f'<h2>{stage["title"]}</h2>'
            f'<span class="price {kind}">{label}</span></header>'
            f'<p class="blurb">{stage.get("summary", "").strip()}</p>'
            f'<ol>{items}</ol></section>')
    return '\n'.join(out)


def build(root, out):
    """Render the whole site into `out`."""
    docs = root / 'docs'
    out.mkdir(parents=True, exist_ok=True)

    version = (root / 'VERSION').read_text().strip() if (root / 'VERSION').exists() else ''
    # git is absent inside the build image, so the commit is passed in.
    commit = (run(['git', 'rev-parse', '--short', 'HEAD'], root)
              or os.environ.get('THREEVN_GIT_COMMIT', '')[:7])
    stamp = datetime.date.today().isoformat()
    meta = f'v{version} · {commit or "no commit"} · built {stamp}'

    written = []
    for source in sorted(docs.glob('*.md')):
        html = rewrite_links(render(source.read_text()))
        name = source.stem + '.html'
        (out / name).write_text(PAGE.format(
            title=source.stem.replace('-', ' ').title(), style=STYLE,
            home='index.html', meta=meta, body=f'<a class="back" href="index.html">&larr; all pages</a>\n{html}',
            stamp=f'v{version} · {commit}'))
        written.append(source.name)

    for source in sorted((docs / 'decisions').glob('*.md')):
        html = rewrite_links(render(source.read_text()))
        (out / f'decisions-{source.stem}.html').write_text(PAGE.format(
            title=source.stem, style=STYLE, home='index.html', meta=meta,
            body=f'<a class="back" href="index.html">&larr; all pages</a>\n{html}',
            stamp=f'v{version} · {commit}'))

    stages = load_course(root)
    lessons = [le for st in stages for le in st.get('lessons', [])]
    for i, lesson in enumerate(lessons):
        body = lesson_page(lesson,
                           lessons[i - 1] if i else None,
                           lessons[i + 1] if i + 1 < len(lessons) else None,
                           len(lessons))
        (out / f'lesson-{lesson["id"]}.html').write_text(PAGE.format(
            title=lesson['title'], style=STYLE, home='index.html', meta=meta,
            body=body, stamp=f'v{version} · {commit}'))

    index = _index_html(docs, written, out, stages, len(lessons))
    (out / 'index.html').write_text(PAGE.format(
        title='The course', style=STYLE, home='index.html', meta=meta,
        body=index, stamp=f'v{version} · {commit}'))

    (out / 'robots.txt').write_text('User-agent: *\nDisallow: /\n')
    return len(list(out.glob('*.html')))


def _index_html(docs, written, out, stages, total):
    """Render the front page: the course first, the manual second."""
    listed = {name for _, items in CURRICULUM for name, _ in items}
    parts = ['<h1>Build a robot the way you build software</h1>']
    parts.append(render(
        'One system that grows, from a simulated arm to a mobile '
        'manipulator that sees and learns. You already know Git, Docker, '
        'testing, CI and observability. This is those skills, pointed at '
        'a machine that moves.'))

    free = sum(1 for st in stages if st.get('price') == 'free')
    paid = [st for st in stages if st.get('price') != 'free']
    cost = f"${sum(int(st['price'].strip('$')) for st in paid)}" if paid else 'free'
    parts.append(render(
        f'**{total} lessons** across {len(stages)} stages. The first '
        f'{free} are free. The rest are {paid[0]["price"]} each — '
        f'{cost} for everything.'))

    parts.append(
        '<div class="note"><strong>This is a live preview of an '
        'unfinished course.</strong> It is generated from the project as '
        'it actually stands, including what does not work yet. Where '
        'something is unproven or failing, the lesson says so.</div>')

    parts.append(ladder_html(stages))

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
        rows = [f'<li><a href="{pathlib.Path(n).stem}.html">{label}</a></li>'
                for n, label in items if n in written]
        if rows:
            parts.append(f'<div class="group"><h3>{heading}</h3><ul>'
                         + ''.join(rows) + '</ul></div>')

    extra = sorted(set(written) - listed)
    if extra:
        rows = ''.join(
            f'<li><a href="{pathlib.Path(n).stem}.html">'
            f'{pathlib.Path(n).stem.replace("-", " ")}</a></li>' for n in extra)
        parts.append('<div class="group"><h3>Everything else</h3><ul>'
                     + rows + '</ul></div>')

    adrs = sorted((docs / 'decisions').glob('*.md'))
    if adrs:
        rows = ''.join(
            f'<li><a href="decisions-{a.stem}.html">{a.stem}</a></li>'
            for a in adrs)
        parts.append('<div class="group"><h3>Decisions</h3><ul>'
                     + rows + '</ul></div>')
    return '\n'.join(parts)


def main(argv=None):
    """Build the static course site."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default='.', help='repository root')
    parser.add_argument('--out', default='site', help='output directory')
    parser.add_argument('--clean', action='store_true')
    args = parser.parse_args(argv)

    root = pathlib.Path(args.root).resolve()
    out = pathlib.Path(args.out).resolve()
    if args.clean and out.exists():
        shutil.rmtree(out)
    count = build(root, out)
    print(f'  {count} pages -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

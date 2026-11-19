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

ONE COURSE, SEVERAL ROBOTS. A student picks the machine they will learn
on (course/robots/*.yaml), and every robot gets its own copy of the
course under site/<robot>/. A lesson's objective, reasoning and terms
are the course and are shared; a robot's version replaces only what has
to change -- the steps, the checkpoint, what goes wrong. A lesson is one
of three kinds:

  * `scope: any`  -- robot-agnostic (Docker, CI, releases); the same page
                     on every robot.
  * `needs: [..]` -- only on robots that have those capabilities; a
                     robot without wheels gets no driving lesson.
  * otherwise     -- written for the reference robot, and for each other
                     robot either given its own version under
                     `flavors: {robot: {steps: ...}}`, or shown as
                     PLANNED: the shared idea, and a link to the same
                     lesson on the reference robot in simulation.

So a robot's coverage is counted, never claimed, the same way stage
durations are summed rather than typed.

A SIDE TRACK is a course file with `kind: track`: unnumbered, filed
after every stage, shown on its own below the course, and left out of a
robot's count. Make your own robot is one: the design story of the 3VN
Arm 1, which every robot's student can take.

Each robot's copy carries its own topic pages, so a topic lists the
lessons that robot's student will actually meet.

Three constraints come from where this is served -- behind the shared
3VNSYSTEMS Caddy, on an unlisted path:

  * Every link is RELATIVE. The site lives under /rc-<something>/ today
    and may live at a bare domain tomorrow, and that move is supposed to
    be a config change rather than a rewrite.
  * Nothing NEEDS JavaScript. The enforced CSP is `script-src 'self'`
    with no 'unsafe-inline', so an inline <script> is dead on arrival.
    Folding (self-check answers, the side map) is <details>; the phone
    Menu is a checkbox. Two same-origin files: nav.js (NAV_JS) makes the
    Course and Reference tabs remember the page last read in each; with
    it blocked they are plain links to each half's front page. theme.js
    (THEME_JS) runs the header's Auto / Light / Dark switch; with it
    blocked the switch is hidden and the system setting decides.
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
#: A `stage` and a `tier` are the numbered path. A `track` is a side
#: track: off that path, unnumbered, filed after every stage, and not in
#: a robot's count of lessons written for it.
STAGE_KINDS = ('stage', 'tier', 'track')
LESSON_FIELDS = ('id', 'title', 'tags', 'minutes', 'objective', 'body',
                 'steps', 'checkpoint')
PROJECT_FIELDS = ('title', 'minutes', 'brief', 'tasks', 'done_when')
TAG_FIELDS = ('slug', 'label', 'description')

#: The topic vocabulary, in course/ beside the stages but not one of them.
TAGS_FILE = 'tags.yaml'

#: How many "often studied alongside" lessons a topic page recommends,
#: after every lesson its own lessons build on.
RELATED_PICKS = 4

#: What a robot -- one flavour of the course -- must carry.
ROBOT_FIELDS = ('id', 'name', 'order', 'price', 'price_note', 'kind', 'has',
                'summary', 'status', 'teaches')
#: What a robot can have, and so what a lesson can need -- with how a
#: page says it. `wheels`, not "mobile": a walking duck is mobile, and
#: the lessons that need wheels are about wheel odometry.
CAPABILITIES = {'arm': 'an arm', 'gripper': 'a gripper', 'wheels': 'wheels',
                'legs': 'legs', 'camera': 'a camera', 'lidar': 'a lidar'}
#: What a robot's version may replace. The objective, the reasoning and
#: the terms ARE the course; the steps are the robot. Concepts given in
#: a robot's version are added to the shared ones, not swapped for them.
LESSON_OVERRIDES = ('steps', 'checkpoint', 'pitfalls', 'quiz', 'exercise',
                    'read', 'note', 'minutes', 'concepts')
PROJECT_OVERRIDES = ('brief', 'tasks', 'done_when', 'hints', 'read', 'note',
                     'minutes')

#: Blueprint, the dark theme's colours. Written out twice in STYLE: once
#: for a reader whose system is dark and who has not picked Light, once
#: for a reader who picked Dark.
DARK = """
    --bg: #0d1322; --panel: #131b2d; --sunken: #0a101c; --line: #26324b;
    --ink: #e8ecf4; --dim: #9ba6bc; --accent: #8ab4ff; --violet: #b9a2ff;
    --teal: #4fd1c1; --warn: #f2b84b; --good: #6fdcae; --bad: #ff8f85;
    --grid: rgba(138, 180, 255, .075); --term: #080d18; --term-ink: #e4e8f1;
    color-scheme: dark;
"""

STYLE = """
/* Two themes: paper by day, blueprint by night. The reader's system
   setting chooses, unless they picked one with the header's switch
   (theme.js sets data-theme on <html>; without it, the system decides).
   The three accents are the strokes of the 3VN mark -- blue, violet, teal. */
:root {
  --bg: #f6f2e8; --panel: #fffcf5; --sunken: #eee8da; --line: #dcd4c1;
  --ink: #1c2230; --dim: #5b6271; --accent: #1d4ed8; --violet: #6d28d9;
  --teal: #0f766e; --warn: #9a4a0b; --good: #0f766e; --bad: #b42318;
  --grid: rgba(29, 78, 216, .08); --term: #141a28; --term-ink: #e4e8f1;
  --serif: "Iowan Old Style", Charter, "Bitstream Charter", "Sitka Text",
           Cambria, Georgia, serif;
  --sans: ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
  --mono: ui-monospace, "SF Mono", SFMono-Regular, Menlo, Consolas, monospace;
  --thread: linear-gradient(90deg, #2563eb, #7c3aed 50%, #14b8a6);
  --gutter: 20px;
  --control: 34px;  /* every control in the header is this tall, so it never moves */
  color-scheme: light;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {@DARK@}
}
:root[data-theme="dark"] {@DARK@}
* { box-sizing: border-box; }
html { -webkit-text-size-adjust: 100%; scrollbar-gutter: stable; }
body {
  margin: 0; background: var(--bg); color: var(--ink);
  font: 16.5px/1.7 var(--sans);
}
::selection { background: color-mix(in srgb, var(--accent) 24%, transparent); }
.wrap { max-width: 880px; margin: 0 auto; padding: 0 var(--gutter) 72px; }
a { color: var(--accent); text-decoration-thickness: 1px; text-underline-offset: 3px; }
a:hover { text-decoration-thickness: 2px; }
h1, h2 { font-family: var(--serif); font-weight: 600; letter-spacing: -.005em;
         text-wrap: balance; }
h1 { font-size: 38px; line-height: 1.15; margin: 0 0 16px; }
h2 { font-size: 25px; line-height: 1.25; margin: 48px 0 14px; }
h3 { font: 600 12px/1.4 var(--mono); margin: 28px 0 10px; color: var(--dim);
     text-transform: uppercase; letter-spacing: .12em; }
h4 { font-size: 16px; margin: 22px 0 6px; }
p { text-wrap: pretty; }
p, li { color: var(--ink); }
hr { border: 0; border-top: 1px solid var(--line); margin: 32px 0; }
code {
  background: var(--sunken); border: 1px solid var(--line); border-radius: 5px;
  padding: .08em .38em; font-size: .84em; font-family: var(--mono);
}
/* A long identifier in a sentence breaks rather than widen a phone's page. */
:not(pre) > code { overflow-wrap: anywhere; }
pre {
  background: var(--term); color: var(--term-ink); border-radius: 10px;
  border-left: 3px solid var(--teal); padding: 15px 18px; overflow-x: auto;
}
pre code { background: none; border: 0; padding: 0; font-size: 13.5px;
           line-height: 1.6; color: inherit; }
table { border-collapse: collapse; width: 100%; margin: 18px 0; font-size: 14.5px;
        display: block; overflow-x: auto; }
th, td { border-bottom: 1px solid var(--line); padding: 9px 12px; text-align: left;
         vertical-align: top; }
th { font: 600 11.5px/1.5 var(--mono); text-transform: uppercase; letter-spacing: .08em;
     color: var(--dim); border-bottom: 2px solid var(--line); }
blockquote {
  margin: 18px 0; padding: 2px 18px; border-left: 3px solid var(--violet);
  color: var(--dim); font-family: var(--serif); font-size: 17.5px;
}

/* --- frame --- */
header.site { border-bottom: 1px solid var(--line); margin-bottom: 40px;
              background: var(--panel); }
header.site::before { content: ""; display: block; height: 3px; background: var(--thread); }
header.site .wrap { padding-top: 16px; padding-bottom: 14px; display: flex;
                    flex-wrap: wrap; align-items: center; gap: 10px 18px; }
.brand { display: flex; align-items: center; gap: 8px; }
.brand a { display: flex; align-items: center; gap: 10px; color: var(--ink);
           text-decoration: none; font: 600 19px/1.2 var(--serif); }
.brand svg { width: 21px; height: 24px; flex: none; }
.mark-dot { fill: var(--ink); }
.badge {
  display: inline-block; padding: 1px 9px; border: 1px solid var(--line);
  border-radius: 999px; font: 500 11px/1.6 var(--mono); color: var(--dim);
  text-transform: uppercase; letter-spacing: .08em;
}
footer.site {
  border-top: 1px solid var(--line); margin-top: 64px; padding-top: 22px;
  color: var(--dim); font-size: 13.5px; display: flex; gap: 12px;
}
footer.site svg { width: 16px; height: 18px; flex: none; margin-top: 3px; }
footer.site p { margin: 0; color: var(--dim); }
footer.site .meta { font: 12px/1.6 var(--mono); margin-top: 4px; }
.back { display: inline-block; margin-bottom: 20px; font-size: 14px; }
.crumbs { font-size: 14px; margin-bottom: 22px; color: var(--dim); }
.crumbs a { color: var(--dim); text-decoration: none; }
.crumbs a:hover { color: var(--accent); text-decoration: underline; }
.eyebrow, .lesson-head {
  font: 600 12px/1.5 var(--mono); letter-spacing: .12em; text-transform: uppercase;
  color: var(--violet); margin-bottom: 10px;
}
.lede { font-size: 18.5px; line-height: 1.6; }

/* folded things: a plus that turns into a cross */
details { border: 1px solid var(--line); border-radius: 10px; margin: 10px 0;
          padding: 0 16px; background: var(--panel); }
details > summary { cursor: pointer; padding: 12px 0 12px 26px; font-weight: 550;
                    list-style: none; position: relative; }
details > summary::-webkit-details-marker { display: none; }
details > summary::before {
  content: "+"; position: absolute; left: 0; top: 11px; width: 18px; height: 18px;
  border-radius: 50%; border: 1px solid var(--line); color: var(--accent);
  font: 600 14px/16px var(--mono); text-align: center; transition: transform .15s;
}
details[open] > summary::before { transform: rotate(45deg); }
details[open] > summary { border-bottom: 1px solid var(--line); margin-bottom: 8px; }
details > div { padding-bottom: 10px; }

.note {
  background: color-mix(in srgb, var(--warn) 8%, var(--panel));
  border: 1px solid color-mix(in srgb, var(--warn) 30%, var(--line));
  border-radius: 10px; padding: 14px 18px; margin: 24px 0; font-size: 15.5px;
}
.note strong { color: var(--warn); }
.group { margin: 26px 0; }
.group ul { list-style: none; padding: 0; margin: 0; border-top: 1px solid var(--line); }
.group li { border-bottom: 1px solid var(--line); }
.group a { display: flex; justify-content: space-between; gap: 12px; padding: 10px 4px;
           color: var(--ink); text-decoration: none; }
.group a::after { content: "\\2192"; color: var(--dim); }
.group a:hover { color: var(--accent); }
.group a:hover::after { color: var(--accent); }

/* --- heroes and drawings --- */
.hero { position: relative; padding: 4px 0 8px; }
.hero::before {
  content: ""; position: absolute; z-index: -1; pointer-events: none;
  top: -40px; bottom: -8px; left: calc(-1 * var(--gutter)); right: calc(-1 * var(--gutter));
  background-image: linear-gradient(var(--grid) 1px, transparent 1px),
                    linear-gradient(90deg, var(--grid) 1px, transparent 1px);
  background-size: 24px 24px;
  -webkit-mask-image: radial-gradient(ellipse at 70% 35%, #000 25%, transparent 72%);
  mask-image: radial-gradient(ellipse at 70% 35%, #000 25%, transparent 72%);
}
.hero h1 { font-size: 46px; line-height: 1.08; }
.hero.split { display: grid; grid-template-columns: minmax(0, 1fr) 230px; gap: 8px 28px;
              align-items: center; }
.hero.split .art { grid-row: 1 / span 3; grid-column: 2; }
.art svg { display: block; width: 100%; height: auto; }
.art .gd { stroke: var(--dim); stroke-width: 1.2; stroke-dasharray: 3 5; fill: none; }
.art .hatch { stroke: var(--dim); stroke-width: 1.2; fill: none; }
.art .ln { stroke: var(--ink); stroke-width: 2.4; fill: var(--panel);
           stroke-linejoin: round; stroke-linecap: round; }
.art .lk { stroke: var(--accent); stroke-width: 9; stroke-linecap: round; fill: none; }
.art .jt { stroke: var(--ink); stroke-width: 2.4; fill: var(--panel); }
.art .hub { fill: var(--ink); }
.art .jaw { stroke: var(--violet); stroke-width: 4; fill: none;
            stroke-linecap: round; stroke-linejoin: round; }
.art .lb { font: 600 9px var(--mono); fill: var(--dim); letter-spacing: .1em; }

.robots { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
          gap: 18px; margin: 30px 0 8px; }
.robot { border: 1px solid var(--line); border-radius: 14px; background: var(--panel);
         overflow: hidden; display: flex; flex-direction: column; }
.robot .art { display: block; padding: 18px 26px 4px; border-bottom: 1px solid var(--line);
              background-color: var(--bg);
              background-image: linear-gradient(var(--grid) 1px, transparent 1px),
                                linear-gradient(90deg, var(--grid) 1px, transparent 1px);
              background-size: 16px 16px; }
.robot .body { padding: 16px 20px 18px; display: flex; flex-direction: column; flex: 1; }
.robot .eyebrow { margin-bottom: 4px; }
.robot h2 { margin: 0 0 6px; font-size: 26px; }
.robot h2 a { color: var(--ink); text-decoration: none; }
.robot h2 a:hover { color: var(--accent); }
.robot .price { margin: 0 0 10px; align-self: flex-start; }
.robot p { font-size: 15px; margin: 0 0 12px; }
.robot .go { margin: auto 0 0; padding-top: 14px; font-size: 14.5px; }
ul.teach { margin: 4px 0 4px; padding: 0 0 0 18px; color: var(--dim); font-size: 14px; }
ul.teach li { color: var(--dim); margin: 3px 0; }
ul.teach li::marker { color: var(--teal); }
.meter { height: 6px; border-radius: 999px; background: var(--sunken);
         overflow: hidden; margin: 2px 0 6px; }
.meter span { display: block; height: 100%; border-radius: 999px;
              background: linear-gradient(90deg, var(--accent), var(--teal)); }
.button { display: inline-block; padding: 8px 15px; border-radius: 999px;
          background: var(--accent); color: var(--bg); font-weight: 600;
          text-decoration: none; margin-right: 10px; }
.button:hover { filter: brightness(1.08); text-decoration: none; }

/* --- anatomy of a lesson --- */
ol.anatomy { list-style: none; padding: 0; margin: 18px 0;
             display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
             gap: 10px; counter-reset: part; }
ol.anatomy li { counter-increment: part; border: 1px solid var(--line); border-radius: 10px;
                background: var(--panel); padding: 12px 14px 12px 48px; position: relative;
                font-size: 14.5px; color: var(--dim); }
ol.anatomy li::before {
  content: counter(part, decimal-leading-zero); position: absolute; left: 14px; top: 12px;
  font: 600 13px/1.7 var(--mono); color: var(--violet);
}
ol.anatomy b { display: block; color: var(--ink); font: 600 16px/1.4 var(--serif); }

/* --- the ladder --- */
.stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
         gap: 10px; margin: 22px 0 8px; }
.stat { background: var(--panel); border: 1px solid var(--line); border-radius: 10px;
        padding: 12px 15px; }
.stat b { display: block; font: 600 27px/1.15 var(--serif); }
.stat span { color: var(--dim); font-size: 13px; }
.ladder { position: relative; margin: 24px 0 8px; }
.ladder::before {
  content: ""; position: absolute; left: 23px; top: 24px; bottom: 40px; width: 2px;
  background: linear-gradient(var(--accent), var(--violet) 55%, var(--teal)); opacity: .4;
}
.stage { display: grid; grid-template-columns: 48px minmax(0, 1fr); gap: 0 16px;
         margin: 0 0 20px; position: relative; }
.stage .num {
  width: 48px; height: 48px; border-radius: 50%; display: flex; align-items: center;
  justify-content: center; background: var(--bg); border: 2px solid currentColor;
  font: 600 19px/1 var(--serif); color: var(--accent); position: relative;
}
.stage:nth-child(3n+2) .num { color: var(--violet); }
.stage:nth-child(3n) .num { color: var(--teal); }
.stage .card { border: 1px solid var(--line); border-radius: 12px; background: var(--panel);
               overflow: hidden; }
.stage header { display: flex; align-items: baseline; gap: 2px 12px; flex-wrap: wrap;
                padding: 14px 18px 0; }
.stage .n { color: var(--dim); font: 600 11.5px/1.5 var(--mono); letter-spacing: .12em;
            text-transform: uppercase; width: 100%; }
.stage h2 { margin: 0; font-size: 22px; }
.stage h2 a { color: var(--ink); text-decoration: none; }
.stage h2 a:hover { color: var(--accent); }
.price { margin-left: auto; font: 500 12px/1.6 var(--mono); padding: 1px 10px;
         white-space: nowrap; border-radius: 999px; border: 1px solid var(--line);
         color: var(--dim); }
h1 .price { display: inline-block; vertical-align: middle; font-weight: 500;
            margin-left: 6px; }
.price.free { color: var(--good); border-color: color-mix(in srgb, var(--good) 45%, var(--line)); }
.price.paid { color: var(--warn); border-color: color-mix(in srgb, var(--warn) 45%, var(--line)); }
.stage .blurb { padding: 8px 18px 0; color: var(--dim); font-size: 15px; margin: 0; }
.stage .facts { padding: 6px 18px 0; color: var(--dim); font: 12px/1.6 var(--mono); }
.stage ol { margin: 10px 0 0; padding: 10px 18px 14px 44px; border-top: 1px dashed var(--line); }
.stage li { margin: 5px 0; }
.stage ol a { color: var(--ink);
              text-decoration-color: color-mix(in srgb, var(--ink) 25%, transparent); }
.stage ol a:hover { color: var(--accent); text-decoration-color: currentColor; }
.stage ol li::marker { color: var(--dim); font: 12px var(--mono); }
.mins { color: var(--dim); font: 12px var(--mono); margin-left: 8px; white-space: nowrap; }
.stage li.proj { list-style: none; margin-left: -22px; margin-top: 10px; }
.stage li.proj::before { content: "\\25C6  "; color: var(--warn); }
.obj { color: var(--dim); font-size: 13.5px; display: block; }

/* --- a lesson --- */
.dots { display: inline-flex; gap: 4px; margin-left: 10px; vertical-align: 1px; }
.dots i { width: 7px; height: 7px; border-radius: 50%; background: var(--line); }
.dots i.past { background: color-mix(in srgb, var(--violet) 55%, var(--line)); }
.dots i.here { background: var(--violet); box-shadow: 0 0 0 3px color-mix(in srgb, var(--violet) 25%, transparent); }
.objective { font: 20px/1.45 var(--serif); color: var(--ink); margin: 0 0 18px; }
.objective b { display: block; font: 600 11.5px/1.6 var(--mono); color: var(--dim);
               text-transform: uppercase; letter-spacing: .12em; margin-bottom: 2px; }
.check {
  background: color-mix(in srgb, var(--good) 8%, var(--panel));
  border: 1px solid color-mix(in srgb, var(--good) 35%, var(--line));
  border-radius: 10px; padding: 14px 18px 14px 52px; margin: 26px 0;
  font-size: 15.5px; position: relative;
}
.check::before {
  content: "\\2713"; position: absolute; left: 17px; top: 15px; width: 22px; height: 22px;
  border-radius: 50%; background: var(--good); color: var(--bg); text-align: center;
  font: 700 13px/22px var(--sans);
}
.check strong { color: var(--good); }
ol.steps { counter-reset: step; list-style: none; padding: 0; margin: 0; }
ol.steps > li { counter-increment: step; position: relative; padding-left: 46px;
                margin: 0 0 22px; }
ol.steps > li::before {
  content: counter(step); position: absolute; left: 0; top: 0; z-index: 1;
  width: 28px; height: 28px; border-radius: 50%; text-align: center;
  font: 600 13px/28px var(--mono); background: var(--accent); color: var(--bg);
}
ol.steps > li:not(:last-child)::after {
  content: ""; position: absolute; left: 13px; top: 32px; bottom: -18px; width: 2px;
  background: var(--line);
}
ol.steps p { margin: 0 0 8px; }
ol.steps pre { margin: 6px 0 8px; }
.expect { color: var(--dim); font-size: 14.5px; }
.expect b { color: var(--good); font-weight: 600; }
.pitfall { border: 1px solid var(--line); border-left: 3px solid var(--bad);
           border-radius: 10px; margin: 12px 0; padding: 12px 16px; background: var(--panel); }
.pitfall p { margin: 4px 0; font-size: 15px; }
.pitfall .sym { color: var(--bad); font-weight: 600; }
.pitfall b { color: var(--dim); font: 600 11.5px/1.6 var(--mono); text-transform: uppercase;
             letter-spacing: .1em; margin-right: 4px; }
dl.terms { margin: 10px 0; }
dl.terms dt { font: 600 17px/1.4 var(--serif); margin-top: 14px; }
dl.terms dd { margin: 2px 0 0 0; color: var(--dim); font-size: 15px; }
.stretch { border: 1px dashed color-mix(in srgb, var(--violet) 50%, var(--line));
           border-radius: 10px; padding: 12px 18px; }
.pager { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 44px; }
.pager a, .pager span { display: block; border: 1px solid var(--line); border-radius: 10px;
                        padding: 12px 16px; text-decoration: none; background: var(--panel);
                        font-weight: 550; }
.pager a:hover { border-color: var(--accent); }
.pager small { display: block; font: 600 11px/1.6 var(--mono); color: var(--dim);
               text-transform: uppercase; letter-spacing: .12em; }
.pager .next { text-align: right; grid-column: 2; }
.pager span { color: var(--dim); background: none; border-style: dashed; }
.pager span:empty { visibility: hidden; }
ul.checklist { list-style: none; padding-left: 0; }
ul.checklist li { padding-left: 30px; position: relative; margin: 9px 0; }
ul.checklist li::before { content: ""; position: absolute; left: 2px; top: 5px;
                          width: 15px; height: 15px; border-radius: 4px;
                          border: 2px solid var(--good); }
table.syllabus td.m { white-space: nowrap; color: var(--dim); font: 12.5px/1.9 var(--mono); }

/* --- topics --- */
ul.tags { list-style: none; display: flex; flex-wrap: wrap; gap: 6px;
          padding: 0; margin: 8px 0 0; }
ul.tags li, .stage ul.tags li { margin: 0; }
.tag { display: inline-block; font-size: 12.5px; line-height: 1.3;
       padding: 3px 10px; border: 1px solid var(--line); border-radius: 999px;
       background: var(--panel); color: var(--accent); text-decoration: none; }
.tag:hover { border-color: var(--accent); }
.tag.on { background: var(--accent); border-color: var(--accent); color: var(--bg); }
.tag .count { color: var(--dim); background: var(--sunken); border-radius: 999px;
              font: 10.5px/1.6 var(--mono); padding: 0 6px; margin-left: 6px; }
.objective + ul.tags { margin: -8px 0 18px; }
.builds { color: var(--dim); font-size: 14.5px; margin: 0 0 18px; }
ul.picks { list-style: none; padding: 0; margin: 0; }
ul.picks > li { border-bottom: 1px solid var(--line); padding: 12px 2px; }
ul.picks > li:last-child { border-bottom: 0; }
ul.picks .title { font-weight: 600; }
.why { color: var(--dim); font-size: 13.5px; display: block; margin-top: 2px; }
.quote { display: block; margin-top: 6px; padding-left: 12px; font: 15.5px/1.5 var(--serif);
         border-left: 3px solid var(--violet); color: var(--ink); }
h2.stage-head { font-size: 20px; }
h2.stage-head a { color: var(--ink); text-decoration: none; }
h2.stage-head a:hover { color: var(--accent); }
h2.stage-head .n { color: var(--dim); font: 600 11.5px var(--mono); letter-spacing: .12em;
                   text-transform: uppercase; margin-right: 8px; }

/* --- the robot picker --- */
details.picker { margin: 0 0 0 auto; padding: 0 14px; border-radius: 999px;
                 position: relative; max-width: 100%; }
details.picker > summary { padding: 0 18px 0 0; font-weight: 500; font-size: 14px;
                           line-height: calc(var(--control) - 2px); white-space: nowrap; }
details.picker > summary::before { display: none; }
details.picker > summary::after {
  content: ""; position: absolute; right: 3px; top: 50%; width: 6px; height: 6px;
  border-right: 2px solid var(--dim); border-bottom: 2px solid var(--dim);
  transform: translateY(-70%) rotate(45deg); transition: transform .15s;
}
details.picker[open] > summary::after { transform: translateY(-30%) rotate(225deg); }
details.picker > summary span { color: var(--dim); margin-left: 6px; font: 12px var(--mono); }
details.picker > summary i { font-style: normal; }
details.picker[open] > summary { border-bottom: 0; margin-bottom: 0; }
details.picker ul {
  list-style: none; margin: 0; padding: 6px 14px; position: absolute; right: 0;
  top: calc(100% + 8px); z-index: 10; min-width: 270px; background: var(--panel);
  border: 1px solid var(--line); border-radius: 12px;
  box-shadow: 0 12px 32px rgba(0, 0, 0, .18);
}
details.picker li + li { border-top: 1px solid var(--line); }
details.picker li a { display: block; padding: 9px 0; text-decoration: none; font-weight: 550; }
details.picker li span { display: block; color: var(--dim); font: 400 12px/1.5 var(--mono); }
.soon { font: 500 10.5px/1.6 var(--mono); color: var(--warn); text-transform: uppercase;
        letter-spacing: .06em; border: 1px solid color-mix(in srgb, var(--warn) 45%, var(--line));
        border-radius: 999px; padding: 0 7px; margin-left: 8px; white-space: nowrap; }

/* --- the frame: course map, page, outline ---
   Three columns on a wide screen, two on a laptop, one on a phone, where
   the map folds behind a Menu button. The button is a checkbox's label,
   because opening a panel without a script takes one. */
.wrap.wide { max-width: 1380px; }
nav.tabs { display: flex; gap: 2px; }
nav.tabs a { display: block; height: var(--control); line-height: var(--control);
             padding: 0 12px; border-radius: 999px; color: var(--dim);
             text-decoration: none; font-size: 14.5px; font-weight: 550; }
nav.tabs a:hover { color: var(--ink); background: var(--sunken); }
nav.tabs a[aria-current] { color: var(--ink); background: var(--sunken);
                           box-shadow: inset 0 0 0 1px var(--line); }
.vh { position: absolute; width: 1px; height: 1px; margin: -1px; padding: 0;
      overflow: hidden; clip: rect(0 0 0 0); white-space: nowrap; border: 0; }
.menu-btn { display: none; align-items: center; gap: 8px; cursor: pointer;
            border: 1px solid var(--line); border-radius: 999px; padding: 0 13px;
            height: var(--control);
            font-size: 14px; font-weight: 550; background: var(--panel); user-select: none; }
.menu-btn i, .menu-btn i::before, .menu-btn i::after {
  display: block; width: 14px; height: 2px; background: currentColor; border-radius: 2px; }
.menu-btn i { position: relative; }
.menu-btn i::before, .menu-btn i::after { content: ""; position: absolute; left: 0; }
.menu-btn i::before { top: -5px; }
.menu-btn i::after { top: 5px; }
.menu-btn .shut, #menu:checked ~ header .menu-btn .open { display: none; }
#menu:checked ~ header .menu-btn .shut { display: inline; }
#menu:focus-visible ~ header .menu-btn { outline: 2px solid var(--accent); outline-offset: 2px; }

/* --- the theme switch: Auto, Light, Dark ---
   Three radios. Only theme.js can keep the choice from page to page, so
   the switch shows only where it ran (it sets data-theme); without it the
   system setting decides, as it always did. */
.theme { display: none; margin: 0 0 0 auto; padding: 2px; border: 1px solid var(--line);
         border-radius: 999px; height: var(--control); gap: 2px; background: var(--panel); }
:root[data-theme] .theme { display: inline-flex; }
details.picker ~ .theme { margin-left: 0; }  /* the picker already pushed right */
.theme legend { position: absolute; width: 1px; height: 1px; overflow: hidden;
                clip: rect(0 0 0 0); white-space: nowrap; }
.theme label { display: grid; place-items: center; width: calc(var(--control) - 6px);
               border-radius: 999px; color: var(--dim); cursor: pointer; }
.theme label:hover { color: var(--ink); background: var(--sunken); }
.theme svg { width: 16px; height: 16px; fill: none; stroke: currentColor; stroke-width: 1.8;
             stroke-linecap: round; }
.theme svg .fill { fill: currentColor; stroke: none; }
.theme input:checked + label { color: var(--ink); background: var(--sunken);
                               box-shadow: inset 0 0 0 1px var(--line); }
.theme input:focus-visible + label { outline: 2px solid var(--accent); outline-offset: 1px; }

.frame { display: grid; grid-template-columns: 236px minmax(0, 1fr); gap: 0 40px;
         align-items: start; }
.frame > main { min-width: 0; max-width: 820px; }
.side, .toc { position: sticky; top: 16px; max-height: calc(100vh - 32px);
              overflow-y: auto; overscroll-behavior: contain; scrollbar-width: thin; }
.side { font-size: 14px; line-height: 1.4; padding: 0 4px 24px 0; }
.side a { color: var(--ink); text-decoration: none; }
.side a:hover { color: var(--accent); }
.side .home { display: block; font: 600 17px/1.3 var(--serif); }
.side .sub { margin: 3px 0 0; color: var(--dim); font: 11.5px/1.5 var(--mono); }
.side .meter { margin: 8px 0 0; height: 4px; }
.side .map-title { margin: 22px 0 6px; font: 600 11px/1.5 var(--mono); color: var(--dim);
                   text-transform: uppercase; letter-spacing: .12em; }
.side details { border: 0; border-radius: 0; background: none; padding: 0; margin: 0; }
.side details > summary { padding: 6px 0 6px 20px; font-weight: 600; font-size: 14px;
                          display: flex; gap: 8px; align-items: baseline; }
.side details > summary::before {
  content: ""; left: 3px; top: 13px; width: 6px; height: 6px; border: 0; border-radius: 0;
  border-right: 2px solid var(--dim); border-bottom: 2px solid var(--dim);
  transform: rotate(-45deg); transition: transform .15s;
}
.side details[open] > summary::before { transform: rotate(45deg); top: 11px; }
.side details[open] > summary { border-bottom: 0; margin-bottom: 0; }
.side summary:hover { color: var(--accent); }
.side summary.here { color: var(--violet); }
.side .sn { flex: none; font: 600 11px/1 var(--mono); color: var(--dim); min-width: 1.2em; }
.side summary.here .sn { color: var(--violet); }
.side ol, .side ul { list-style: none; margin: 2px 0 8px 9px; padding: 0;
                     border-left: 1px solid var(--line); }
.side li a { display: flex; gap: 8px; padding: 4px 6px 4px 12px; margin-left: -1px;
             border-left: 2px solid transparent; border-radius: 0 6px 6px 0; }
.side li a .n { flex: none; min-width: 1.3em; color: var(--dim); font: 11px/1.75 var(--mono); }
.side li a[aria-current] {
  border-left-color: var(--violet); color: var(--violet); font-weight: 600;
  background: color-mix(in srgb, var(--violet) 9%, transparent);
}
.side li a[aria-current] .n { color: var(--violet); }
.side li.planned a { color: color-mix(in srgb, var(--dim) 75%, var(--bg)); }
.side li.planned a[aria-current] { color: var(--violet); }
.side li.proj .n { color: var(--warn); }
.side ul.pick { border-left: 0; margin: 10px 0 0; display: grid; gap: 6px; }
.side ul.pick a { display: block; margin: 0; padding: 9px 12px; border: 1px solid var(--line);
                  border-radius: 10px; background: var(--panel); }
.side ul.pick a:hover { border-color: var(--accent); }
.side ul.pick b { display: block; font: 600 15px/1.3 var(--serif); }
.side ul.pick span { display: block; color: var(--dim); font: 11px/1.5 var(--mono); }
.side ul.pick .meter { margin: 6px 0 0; height: 4px; }
.side ul.pick .meter span { height: 100%; }
.side ul.pick + .sub { margin-top: 10px; }
.side ul.flat { border-left: 0; margin-left: 0; }
.side ul.flat a { padding-left: 0; border-left: 0; }
.side ul.flat a[aria-current] { background: none; }

.toc { display: none; font-size: 13.5px; line-height: 1.4; }
.toc b { display: block; margin: 0 0 8px; font: 600 11px/1.5 var(--mono); color: var(--dim);
         text-transform: uppercase; letter-spacing: .12em; }
.toc ul { list-style: none; margin: 0; padding: 0; border-left: 1px solid var(--line); }
.toc a { display: block; padding: 4px 0 4px 12px; color: var(--dim); text-decoration: none; }
.toc a:hover { color: var(--accent); }
h2[id] { scroll-margin-top: 16px; }

@media (min-width: 1240px) {
  .frame { grid-template-columns: 252px minmax(0, 1fr); gap: 0 52px; }
  .frame.has-toc { grid-template-columns: 252px minmax(0, 1fr) 188px; }
  .toc { display: block; }
}
@media (max-width: 899px) {
  /* Two fixed rows on every page, whatever it has: the brand and Menu,
     then the tabs and the robot. A page without Menu or a robot leaves
     its cell empty rather than letting the rest move up. */
  header.site .wrap { display: grid; gap: 10px 12px; align-items: center;
                      grid-template-columns: auto minmax(0, 1fr) auto;
                      grid-template-areas: "brand theme menu" "tabs picker picker"; }
  .brand { grid-area: brand; min-height: var(--control); }
  .theme { grid-area: theme; justify-self: end; }
  nav.tabs { grid-area: tabs; }
  details.picker { grid-area: picker; justify-self: end; min-width: 0; max-width: 100%;
                   margin: 0; }
  details.picker > summary { overflow: hidden; text-overflow: ellipsis; }
  .menu-btn { grid-area: menu; justify-self: end; }
  .frame { display: block; }
  .menu-btn { display: inline-flex; }
  .side { display: none; position: static; max-height: none; overflow: visible;
          border: 1px solid var(--line); border-radius: 12px; background: var(--panel);
          padding: 14px 16px 6px; margin: -18px 0 30px; }
  #menu:checked ~ .wrap .side { display: block; }
}

@media (max-width: 640px) {
  :root { --gutter: 16px; }
  body { font-size: 16px; }
  h1 { font-size: 30px; }
  .hero h1 { font-size: 34px; }
  h2 { font-size: 22px; }
  .hero.split { grid-template-columns: 1fr; }
  .hero.split .art { grid-row: auto; grid-column: auto; max-width: 220px; order: -1; }
  .ladder::before { left: 17px; }
  .stage { grid-template-columns: 36px minmax(0, 1fr); gap: 0 10px; }
  .stage .num { width: 36px; height: 36px; font-size: 15px; }
  .stage ol { padding-left: 36px; }
  .pager { grid-template-columns: 1fr; }
  .pager .next { grid-column: 1; }
  footer.site { flex-direction: column; }
  details.picker > summary span, details.picker > summary i { display: none; }
}
@media (max-width: 460px) {
  .badge { display: none; }  /* room in the top row for the theme switch */
}
@media (max-width: 360px) {
  nav.tabs a { padding: 0 9px; }
  details.picker { padding: 0 10px; }
}
@media (max-width: 400px) {
  /* The brand, the theme switch and Menu share the top row: on the
     narrowest phones Menu is its icon alone (the word is still read out). */
  header.site .wrap { column-gap: 8px; }
  .menu-btn { padding: 0 11px; }
  .menu-btn span { position: absolute; width: 1px; height: 1px; overflow: hidden;
                   clip: rect(0 0 0 0); white-space: nowrap; }
  .theme label { width: 26px; }
}
@media (max-width: 340px) {
  .theme label { width: 22px; }
}
""".replace('@DARK@', DARK)

# The 3VN mark: the logo's three strokes and its dot (brand/3vn-mark.svg
# in the homepage repository), the dot recoloured to follow the theme.
MARK = (
    '<svg viewBox="0 0 260 290" aria-hidden="true">'
    '<g fill="none" stroke-width="23" stroke-linecap="round" stroke-linejoin="round">'
    '<path stroke="#2563eb" d="M23,25 C23.0,28.3 22.3,38.3 23,45 C23.7,51.7 25.3,58.3 27,65 C28.7,71.7 30.3,78.3 33,85 C35.7,91.7 39.2,98.3 43,105 C46.8,111.7 51.3,118.3 56,125 C60.7,131.7 66.0,138.3 71,145 C76.0,151.7 81.3,158.3 86,165 C90.7,171.7 94.8,178.3 99,185 C103.2,191.7 107.7,198.3 111,205 C114.3,211.7 116.7,218.7 119,225 C121.3,231.3 124.0,240.0 125,243"/>'
    '<path stroke="#14b8a6" d="M234,25 C233.8,28.3 233.5,38.3 233,45 C232.5,51.7 232.5,58.3 231,65 C229.5,71.7 226.8,78.3 224,85 C221.2,91.7 217.8,98.3 214,105 C210.2,111.7 205.5,118.3 201,125 C196.5,131.7 192.0,138.3 187,145 C182.0,151.7 175.3,158.3 171,165 C166.7,171.7 164.2,178.3 161,185 C157.8,191.7 155.3,198.3 152,205 C148.7,211.7 144.2,218.7 141,225 C137.8,231.3 134.3,240.0 133,243"/>'
    '<path stroke="#7c3aed" d="M129,15 C128.0,18.3 124.8,28.3 123,35 C121.2,41.7 118.8,48.3 118,55 C117.2,61.7 117.5,68.3 118,75 C118.5,81.7 119.5,88.3 121,95 C122.5,101.7 125.0,108.3 127,115 C129.0,121.7 131.3,128.3 133,135 C134.7,141.7 136.0,148.3 137,155 C138.0,161.7 138.7,168.3 139,175 C139.3,181.7 139.3,188.3 139,195 C138.7,201.7 138.0,208.3 137,215 C136.0,221.7 134.3,229.7 133,235 C131.7,240.3 129.7,245.0 129,247"/>'
    '</g><circle class="mark-dot" cx="128.5" cy="254" r="18.5"/></svg>')


def _hatch(x0, x1, y, step=12, depth=9):
    """Ground hatching under a line, the way a drawing shows a fixed surface."""
    return ''.join(f'M{x},{y} l-{depth},{depth} ' for x in range(x0, x1 + 1, step))


# A line drawing of each robot, in the style of a figure in a manual. Pure
# markup coloured by the stylesheet, so it follows the theme. A robot
# without one simply gets no picture.
ROBOT_ART = {
    '3vn-arm1': (
        '<svg viewBox="0 0 240 170" role="img" aria-label="Line drawing of the '
        '3VN Arm 1 on its mobile base">'
        '<text class="lb" x="8" y="16">FIG. 1</text>'
        '<line class="gd" x1="8" y1="156" x2="232" y2="156"/>'
        '<path class="gd" d="M81.6,93.8 A25,25 0 0 1 112.5,73.8"/>'
        '<rect class="ln" x="42" y="116" width="128" height="26" rx="8"/>'
        '<rect class="ln" x="88" y="102" width="36" height="14" rx="3"/>'
        '<line class="lk" x1="106" y1="98" x2="130" y2="46"/>'
        '<line class="lk" x1="130" y1="46" x2="186" y2="64"/>'
        '<circle class="jt" cx="106" cy="98" r="7.5"/>'
        '<circle class="jt" cx="130" cy="46" r="6.5"/>'
        '<circle class="jt" cx="186" cy="64" r="5.5"/>'
        '<path class="jaw" d="M192,61 L205,56 L215,63"/>'
        '<path class="jaw" d="M192,68 L203,77 L214,77"/>'
        '<circle class="jt" cx="68" cy="144" r="11"/><circle class="hub" cx="68" cy="144" r="2.5"/>'
        '<circle class="jt" cx="144" cy="144" r="11"/><circle class="hub" cx="144" cy="144" r="2.5"/>'
        '</svg>'),
    'so101': (
        '<svg viewBox="0 0 240 170" role="img" aria-label="Line drawing of the '
        'SO-101 arm clamped to the edge of a desk">'
        '<text class="lb" x="8" y="16">FIG. 2</text>'
        f'<path class="hatch" d="{_hatch(20, 128, 146)}"/>'
        '<path class="ln" d="M8,132 H132 V146 H8"/>'
        '<path class="ln" style="fill:none" d="M112,126 H142 V154 H122"/>'
        '<line class="ln" x1="122" y1="154" x2="122" y2="160"/>'
        '<path class="gd" d="M57.4,89.7 A25,25 0 0 1 90.6,70.5"/>'
        '<rect class="ln" x="60" y="112" width="58" height="20" rx="4"/>'
        '<rect class="ln" x="70" y="100" width="26" height="12" rx="3"/>'
        '<line class="lk" x1="82" y1="94" x2="104" y2="40"/>'
        '<line class="lk" x1="104" y1="40" x2="160" y2="58"/>'
        '<line class="lk" x1="160" y1="58" x2="172" y2="62"/>'
        '<circle class="jt" cx="82" cy="94" r="7.5"/>'
        '<circle class="jt" cx="104" cy="40" r="6.5"/>'
        '<circle class="jt" cx="160" cy="58" r="5.5"/>'
        '<rect class="ln" x="171" y="55" width="12" height="15" rx="2.5"/>'
        '<path class="jaw" d="M184,58 L207,56 L212,63"/>'
        '<path class="jaw" d="M184,67 L199,79 L208,79"/>'
        '</svg>'),
}


def robot_art(robot, href=None):
    """The robot's drawing, linked to its course when `href` is given."""
    svg = ROBOT_ART.get(robot['id'])
    if not svg:
        return ''
    if href:
        return f'<a class="art" href="{href}" tabindex="-1">{svg}</a>'
    return f'<div class="art">{svg}</div>'

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="robots" content="noindex, nofollow">
<meta name="color-scheme" content="light dark">
<title>{title} — 3VN Robotics</title>
<style>{style}</style>
<script src="{theme_js}"></script>
<script src="{script}" defer></script>
</head>
<body>
{toggle}<header class="site"><div class="wrap wide">
  <div class="brand"><a href="{home}">{mark}3VN Robotics</a><span class="badge">preview</span></div>
  <nav class="tabs" aria-label="Site">{tabs}</nav>
  {picker}{theme}{menu}
</div></header>
<div class="wrap{wide}">
{body}
<footer class="site">
  {mark}
  <div><p>Generated from the repository's own course and documentation, including
  the parts that do not work yet. A <a href="https://3vnsystems.com/">3VN Systems</a> project.</p>
  <p class="meta">{meta}</p></div>
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


def load_robots(root):
    """
    Load the robots the course can be taken on, from course/robots/*.yaml,
    in their declared order.

    A subdirectory rather than course/*.yaml, because every top-level YAML
    file there is a stage.
    """
    folder = root / 'course' / 'robots'
    if not folder.is_dir():
        return []
    robots = [yaml.safe_load(f.read_text()) for f in sorted(folder.glob('*.yaml'))]
    return sorted(robots, key=lambda r: (r.get('order', 99), str(r.get('id'))))


def reference_of(robots):
    """The robot the default lesson text is written for."""
    return next(r for r in robots if r.get('reference'))


def resolve(base, robot, ref_id, overrides):
    """
    One lesson or project as THIS robot's student sees it.

    Ready when it is robot-agnostic, written for this robot alone, the
    reference robot's own, or given a version for this robot. Planned
    otherwise: the shared idea is real, the steps do not exist yet.
    """
    item = dict(base)
    rid = robot['id']
    variant = (base.get('flavors') or {}).get(rid)
    if rid == ref_id or base.get('scope') == 'any' or base.get('only'):
        item['status'] = 'ready'
    elif variant:
        for key in overrides:
            if key not in variant:
                continue
            if key == 'concepts':
                item[key] = list(base.get('concepts') or []) + list(variant[key])
            else:
                item[key] = variant[key]
        item['status'] = 'ready'
    else:
        item['status'] = 'planned'
    return item


def flavor_view(stages, robot, ref):
    """
    The whole course for one robot: lessons it cannot do are set aside
    with the reason, the rest are resolved and renumbered.
    """
    has = set(robot.get('has') or [])
    view, number = [], 0
    for stage in stages:
        st = dict(stage, robot=robot, ref=ref)
        lessons, skipped = [], []
        for base in stage.get('lessons') or []:
            if base.get('only') and robot['id'] not in base['only']:
                continue
            missing = set(base.get('needs') or []) - has
            if missing:
                skipped.append((base, sorted(missing)))
                continue
            lessons.append(resolve(base, robot, ref['id'], LESSON_OVERRIDES))
        for position, lesson in enumerate(lessons, 1):
            number += 1
            lesson.update(index=number, position=position, stage=st)
        project = resolve(stage['project'], robot, ref['id'], PROJECT_OVERRIDES)
        project['stage'] = st
        st.update(lessons=lessons, skipped=skipped, project=project)
        st['minutes'] = (sum(int(le.get('minutes') or 0) for le in lessons)
                         + int(project.get('minutes') or 0))
        view.append(st)
    return view


def coverage(view):
    """
    (ready, total) lessons on one robot's numbered course. Counted, never
    claimed. A side track is off that path, so it is not in the count.
    """
    lessons = [le for st in view if not is_track(st) for le in st['lessons']]
    return sum(le['status'] == 'ready' for le in lessons), len(lessons)


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

def validate(stages, root, tags=None, robots=None):
    """
    Return every problem with the curriculum, as a list of strings.

    Empty means valid. Checks the shape of each stage, lesson and project,
    and -- more usefully -- that what a lesson points at exists: Make
    targets, docs, topic tags, the lessons it builds on, and the
    uniqueness of ids and glossary terms. A robot's version of a lesson
    is held to exactly the same checks as the lesson.
    """
    errors = []
    docs = root / 'docs'
    if (docs / 'reference.md').exists():
        errors.append('docs/reference.md: reference.html is the reference\'s own '
                      'index, which would overwrite it -- rename the doc')
    targets = make_targets(root)
    seen_ids, seen_terms = {}, {}
    order = {le.get('id'): le.get('index')
             for st in stages for le in (st.get('lessons') or [])}
    robots = robots or []
    robot_ids = {r.get('id') for r in robots}
    refs = [r for r in robots if r.get('reference')]
    ref_id = refs[0].get('id') if len(refs) == 1 else None
    variant_terms = []      # (where, robot, term), checked once every base term is known
    needed = set()          # capabilities some lesson for everyone needs

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

    def check_lesson_content(lwhere, lesson):
        """Everything a lesson points at -- for a lesson or a robot's version of one."""
        for i, step in enumerate(lesson.get('steps') or [], 1):
            if not isinstance(step, dict) or not step.get('do'):
                errors.append(f'{lwhere}: step {i} has no `do`')
                continue
            check_shell(f'{lwhere} step {i}', step.get('run', ''))
            check_shell(f'{lwhere} step {i}', code_in(step.get('do', '')))
            check_markdown(f'{lwhere} step {i}', step.get('do', ''))

        for field in ('body', 'checkpoint', 'note'):
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

        if 'minutes' in lesson and (not isinstance(lesson['minutes'], int)
                                    or lesson['minutes'] <= 0):
            errors.append(f'{lwhere}: `minutes` must be a positive integer')

    def check_project_content(pwhere, project):
        for i, task in enumerate(project.get('tasks') or [], 1):
            text = task.get('do') if isinstance(task, dict) else task
            if not text:
                errors.append(f'{pwhere}: task {i} is empty')
            if isinstance(task, dict):
                check_shell(f'{pwhere} task {i}', task.get('run', ''))
            check_shell(f'{pwhere} task {i}', code_in(text or ''))
        for field in ('brief', 'note'):
            check_markdown(pwhere, project.get(field, ''))
        for name in project.get('read') or []:
            check_doc(pwhere, name)

    def check_flavors(where, item, required, overrides, content):
        """The robot-specific parts of a lesson or project."""
        scope = item.get('scope')
        if scope not in (None, 'any'):
            errors.append(f'{where}: scope {scope!r} is not "any"')
        if scope == 'any' and (item.get('flavors') or item.get('needs')
                               or item.get('only')):
            errors.append(f'{where}: `scope: any` means the same on every robot, '
                          f'so it cannot also have `flavors`, `needs` or `only`')
        for cap in item.get('needs') or []:
            if cap not in CAPABILITIES:
                errors.append(f'{where}: needs {cap!r}, which is not one of '
                              f'{", ".join(CAPABILITIES)}')
        for rid in item.get('only') or []:
            if rid not in robot_ids:
                errors.append(f'{where}: `only` names robot {rid!r}, '
                              f'which course/robots/ does not define')
        if not item.get('only'):
            needed.update(item.get('needs') or [])
        for rid, variant in (item.get('flavors') or {}).items():
            vwhere = f'{where} on {rid}'
            if rid not in robot_ids:
                errors.append(f'{vwhere}: no such robot in course/robots/')
                continue
            if rid == ref_id:
                errors.append(f'{vwhere}: the reference robot\'s version is the '
                              f'lesson itself; edit that instead')
                continue
            if not isinstance(variant, dict) or not variant.get(required):
                errors.append(f'{vwhere}: a robot\'s version must give its own '
                              f'`{required}`; leave it out to show it as planned')
                continue
            for key in variant:
                if key not in overrides:
                    errors.append(f'{vwhere}: `{key}` cannot differ by robot '
                                  f'(only {", ".join(overrides)} can)')
            content(vwhere, variant)
            for concept in variant.get('concepts') or []:
                term = str(concept.get('term', '')).strip()
                if not term or not concept.get('meaning'):
                    errors.append(f'{vwhere}: a concept needs `term` and `meaning`')
                    continue
                variant_terms.append((vwhere, rid, term))

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

    # ---- the robots
    if not robots:
        errors.append('course/robots/: no robots found')
    elif len(refs) != 1:
        errors.append(f'course/robots/: exactly one robot must be '
                      f'`reference: true`, found {len(refs)}')
    seen_robots = set()
    for robot in robots:
        rwhere = f'robot {robot.get("id", "?")}'
        need(rwhere, robot, ROBOT_FIELDS)
        rid = str(robot.get('id', ''))
        if rid in seen_robots:
            errors.append(f'{rwhere}: id used by another robot')
        seen_robots.add(rid)
        if rid and not re.fullmatch(r'[a-z0-9][a-z0-9-]*', rid):
            errors.append(f'{rwhere}: id must be lowercase letters, digits and '
                          f'dashes -- it becomes a directory and a URL')
        for cap in robot.get('has') or []:
            if cap not in CAPABILITIES:
                errors.append(f'{rwhere}: has {cap!r}, which is not one of '
                              f'{", ".join(CAPABILITIES)}')
        for field in ('summary', 'status', 'price_note'):
            check_markdown(rwhere, robot.get(field, ''))
        for name in robot.get('read') or []:
            check_doc(rwhere, name)

    # ---- the course
    expected, seen_track = 0, None
    for stage in stages:
        where = f'stage {stage.get("id", expected + 1)}'
        if stage.get('kind') not in STAGE_KINDS:
            errors.append(f'{where}: kind {stage.get("kind")!r} is not one of '
                          f'{", ".join(STAGE_KINDS)}')
        if is_track(stage):
            # Off the numbered path: after every stage, and unnumbered.
            seen_track = stage.get('id')
            need(where, stage, tuple(f for f in STAGE_FIELDS if f != 'number'))
            if 'number' in stage:
                errors.append(f'{where}: a side track is not numbered; '
                              f'remove `number`')
        else:
            expected += 1
            need(where, stage, STAGE_FIELDS)
            if seen_track:
                errors.append(f'{where}: comes after side track {seen_track}; '
                              f'side tracks go after every numbered stage')
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

            check_lesson_content(lwhere, lesson)
            check_flavors(lwhere, lesson, 'steps', LESSON_OVERRIDES,
                          check_lesson_content)

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
            check_project_content(pwhere, project)
            if project.get('needs') or project.get('only'):
                errors.append(f'{pwhere}: every robot gets every project, so a '
                              f'project cannot have `needs` or `only`')
            check_flavors(pwhere, project, 'tasks', PROJECT_OVERRIDES,
                          check_project_content)

    for name in known:
        if name not in used:
            errors.append(f'tag {name}: in course/{TAGS_FILE}, but no lesson carries it')

    # ---- across both
    per_robot = {}
    for vwhere, rid, term in variant_terms:
        key = term.lower()
        mine = per_robot.setdefault(rid, {})
        if key in seen_terms:
            errors.append(f'{vwhere}: glossary term {term!r} is already defined '
                          f'by lesson {seen_terms[key]}')
        elif key in mine:
            errors.append(f'{vwhere}: glossary term {term!r} is already defined '
                          f'for this robot in {mine[key]}')
        mine[key] = vwhere
    if ref_id:
        missing = needed - set(refs[0].get('has') or [])
        if missing:
            errors.append(f'robot {ref_id}: the reference robot must be able to '
                          f'take every lesson, but lacks {", ".join(sorted(missing))}')
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
    if is_track(stage):
        return 'Side track'
    return 'Tier' if stage.get('kind') == 'tier' else 'Stage'


def is_track(stage):
    """A side track: off the numbered path, taken after it or alongside it."""
    return stage.get('kind') == 'track'


def stage_name(stage):
    """How a page names a stage: "Stage 2", "Tier 5", or "Side track"."""
    if is_track(stage):
        return stage_word(stage)
    return f'{stage_word(stage)} {stage["number"]}'


def stage_mark(stage, width=1):
    """The short mark beside a stage in a map or on the ladder: its number, or + for a track."""
    return '+' if is_track(stage) else f'{stage["number"]:0{width}d}'


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
    return (f'{stage_name(stage)} &middot; lesson '
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
        return (f'<a class="prev" href="{href}"><small>&larr; Previous</small>'
                f'{esc(title)}</a>' if arrow_left
                else f'<a class="next" href="{href}"><small>Next &rarr;</small>'
                     f'{esc(title)}</a>')
    prev = link(sequence[position - 1], True) if position else '<span></span>'
    nxt = (link(sequence[position + 1], False) if position + 1 < len(sequence)
           else '<span class="next"><small>That is all</small>'
                'The end of the course</span>')
    return f'<div class="pager">{prev}{nxt}</div>'


def soon(item):
    """The pill on a lesson or project not yet written for this robot."""
    return '<span class="soon">planned</span>' if item.get('status') == 'planned' else ''


def robot_notes(item, page):
    """
    What a lesson or project says about THIS robot: its own note, or --
    when its version does not exist yet -- that, and where to take the
    lesson meanwhile.
    """
    stage = item['stage']
    robot, ref = stage.get('robot'), stage.get('ref')
    out = []
    if robot and item.get('note'):
        out.append(f'<div class="note"><strong>On the {esc(robot["name"])}:</strong> '
                   f'{inline(item["note"])}</div>')
    if robot and item.get('status') == 'planned':
        out.append(
            f'<div class="note"><strong>Not written for the {esc(robot["name"])} '
            f'yet.</strong> The idea above is the same on every robot, though '
            f'it is told through the {esc(ref["name"])}; the steps are not. '
            f'Until they exist, take it on the '
            f'<a href="../{ref["id"]}/{page}">{esc(ref["name"])}</a> in '
            f'simulation, which needs no hardware.</div>')
    return out


def picker_html(robot, robots, pages, counts, name):
    """
    The robot chooser at the top of every course page.

    No script needed: a <details> that opens into links, each to
    THIS page on the other robot where it exists there, and to that
    robot's front page where it does not (a vacuum has no grasp lesson).
    """
    items = []
    for other in robots:
        if other['id'] == robot['id']:
            continue
        target = name if name in pages[other['id']] else 'index.html'
        ready, total = counts[other['id']]
        items.append(f'<li><a href="../{other["id"]}/{target}">{esc(other["name"])}'
                     f'<span>{esc(other["price"])} &middot; {ready} of {total} '
                     f'lessons ready</span></a></li>')
    items.append('<li><a href="../index.html">Compare the robots</a></li>')
    return (f'<details class="picker"><summary><i>Robot: </i><b>{esc(robot["name"])}</b>'
            f'<span>{esc(robot["price"])}</span></summary>'
            f'<ul>{"".join(items)}</ul></details>')


def crumbs(stage):
    """Breadcrumb back to the course and the stage."""
    return (f'<div class="crumbs"><a href="index.html">The course</a> / '
            f'<a href="stage-{stage["id"]}.html">{stage_name(stage)}: '
            f'{esc(stage["title"])}</a></div>')


# ------------------------------------------------------------- navigation
#
# The site is two things: a course you follow in order, and the project's
# reference you look things up in. The header names the two; a map down
# the side shows where this page sits in whichever one it belongs to;
# and a long page lists its own sections on the right. All of it is
# links and <details>, so it needs no script.

def tabs_html(active, course, reference):
    """
    The header's two halves of the site, the one this page is in marked.

    Each links to its half's front page. NAV_JS, where it runs, points the
    other one at the page last read there instead.
    """
    out = []
    for key, label, href in (('course', 'Course', course),
                             ('reference', 'Reference', reference)):
        here = ' aria-current="true"' if key == active else ''
        out.append(f'<a href="{href}" data-tab="{key}"{here}>{label}</a>')
    return ''.join(out)


#: The tabs' script, written beside the pages as nav.js. The CSP
#: allows a same-origin file (`script-src 'self'`), never an inline one.
#: It only improves the tabs; with it blocked or storage off, every link
#: still goes where its href says.
NAV_JS = r"""// 3VN Robotics course site: the Course and Reference tabs remember the
// page last read in each half, and how far down it, for this browser tab
// (sessionStorage). Switching back lands where the reader left off.
// Without this file the tabs go to each half's front page.
(function () {
  'use strict';
  var tabs = document.querySelectorAll('nav.tabs a[data-tab]');
  var brand = document.querySelector('.brand a');
  var store;
  try {
    store = window.sessionStorage;
    store.setItem('3vn-probe', '1');
    store.removeItem('3vn-probe');
  } catch (e) {
    return;  // storage off or full: plain links it is
  }
  if (!tabs.length || !brand) return;

  // Keyed by where the site is served, so two copies on one host never mix.
  var root = new URL(brand.getAttribute('href'), location.href).pathname
    .replace(/[^\/]*$/, '');
  var here = null;
  tabs.forEach(function (a) {
    if (a.hasAttribute('aria-current')) here = a.getAttribute('data-tab');
  });
  if (!here) return;
  var path = location.pathname;
  // The header does not follow the page, so a reader scrolls back up to
  // reach the tabs: where the page is when it is left says nothing. What
  // is kept is where the reader last stopped, anywhere below the top.
  var TOP = 300;
  var STOPPED = 700;  // ms without scrolling

  function key(name) { return '3vn-robotics ' + root + ' ' + name; }
  function read(name) {
    try { return JSON.parse(store.getItem(key(name))); } catch (e) { return null; }
  }
  function write(name, value) {
    try { store.setItem(key(name), JSON.stringify(value)); } catch (e) { /* full */ }
  }
  var last = read(here);
  var y = last && last.path === path && last.y > 0 ? last.y : 0;
  function remember() { write(here, { path: path, y: y }); }
  function point() {
    // The open tab keeps its own front page; the other goes back to
    // whatever was last read there.
    tabs.forEach(function (a) {
      var tab = a.getAttribute('data-tab');
      var last = tab === here ? null : read(tab);
      if (last && typeof last.path === 'string' && last.path.indexOf(root) === 0) {
        a.setAttribute('href', last.path);
      }
    });
  }

  // Arrived by a tab: put the reader back where they were on this page.
  // Back, forward and reload are the browser's to place; they keep the
  // spot. Any other way in is a new visit, which starts at the top.
  var arrived = read('arrived');
  store.removeItem(key('arrived'));
  var how = performance.getEntriesByType ?
    (performance.getEntriesByType('navigation')[0] || {}).type : '';
  if (arrived === path && y > 0) {
    window.scrollTo(0, y);
  } else if (arrived !== path && how !== 'back_forward' && how !== 'reload') {
    y = 0;
  }
  remember();
  point();

  var timer = null;
  window.addEventListener('scroll', function () {
    clearTimeout(timer);
    timer = setTimeout(function () {
      if (window.scrollY > TOP) {
        y = Math.round(window.scrollY);
        remember();
      }
    }, STOPPED);
  }, { passive: true });

  document.addEventListener('click', function (event) {
    if (event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey ||
        event.altKey) return;
    var a = event.target.closest && event.target.closest('nav.tabs a[data-tab]');
    if (a && a.getAttribute('data-tab') !== here) {
      write('arrived', new URL(a.href).pathname);
    }
  });
  window.addEventListener('pagehide', remember);
  document.addEventListener('visibilitychange', function () {
    if (document.visibilityState === 'hidden') remember();
  });
  // Back to a page kept in memory (the back button): it is the latest
  // page of its half again, and the other tab may have moved on.
  window.addEventListener('pageshow', function (event) {
    if (event.persisted) { remember(); point(); }
  });
})();
"""


#: The header's theme switch. Hidden unless theme.js ran (see STYLE).
#: autocomplete=off, or Back would restore a stale choice over the one
#: theme.js reads from storage.
THEME_SWITCH = (
    '<fieldset class="theme"><legend>Theme</legend>'
    '<input type="radio" name="theme" id="theme-auto" value="auto" class="vh" autocomplete="off">'
    '<label for="theme-auto" title="Auto: follow the system"><span class="vh">Auto</span>'
    '<svg viewBox="0 0 16 16" aria-hidden="true"><circle cx="8" cy="8" r="6"/>'
    '<path class="fill" d="M8 2a6 6 0 0 1 0 12z"/></svg></label>'
    '<input type="radio" name="theme" id="theme-light" value="light" class="vh" autocomplete="off">'
    '<label for="theme-light" title="Light"><span class="vh">Light</span>'
    '<svg viewBox="0 0 16 16" aria-hidden="true"><circle cx="8" cy="8" r="3"/>'
    '<path d="M8 1v1.5M8 13.5V15M1 8h1.5M13.5 8H15M3 3l1.1 1.1M11.9 11.9 13 13'
    'M3 13l1.1-1.1M11.9 4.1 13 3"/></svg></label>'
    '<input type="radio" name="theme" id="theme-dark" value="dark" class="vh" autocomplete="off">'
    '<label for="theme-dark" title="Dark"><span class="vh">Dark</span>'
    '<svg viewBox="0 0 16 16" aria-hidden="true">'
    '<path d="M13.5 9.6A5.8 5.8 0 0 1 6.4 2.5a5.8 5.8 0 1 0 7.1 7.1z"/></svg></label>'
    '</fieldset>')

#: The theme switch's script, written beside the pages as theme.js. It is
#: loaded in <head> without `defer` so the chosen theme is on <html>
#: before the first paint; a deferred script would flash the other theme.
#: localStorage, so the choice outlasts the browser tab. Without it the
#: switch stays hidden and the system setting decides.
THEME_JS = r"""// 3VN Robotics course site: the header's Auto / Light / Dark switch.
// Sets data-theme on <html>, which the stylesheet follows, and keeps the
// choice in localStorage. Without this file the system setting decides.
(function () {
  'use strict';
  var root = document.documentElement;
  var KEY = '3vn-robotics theme';
  function stored() {
    var value = null;
    try { value = window.localStorage.getItem(KEY); } catch (e) { /* off */ }
    return value === 'light' || value === 'dark' ? value : 'auto';
  }
  function show(theme) {
    root.setAttribute('data-theme', theme);
    var input = document.getElementById('theme-' + theme);
    if (input) input.checked = true;
  }
  show(stored());

  document.addEventListener('DOMContentLoaded', function () {
    show(root.getAttribute('data-theme'));
    document.addEventListener('change', function (event) {
      var input = event.target;
      if (input.name !== 'theme') return;
      show(input.value);
      try {
        if (input.value === 'auto') window.localStorage.removeItem(KEY);
        else window.localStorage.setItem(KEY, input.value);
      } catch (e) { /* storage off: this page only */ }
    });
  });
  // Another tab changed it, or Back brought this page from the cache.
  window.addEventListener('storage', function (event) {
    if (event.key === KEY || event.key === null) show(stored());
  });
  window.addEventListener('pageshow', function (event) {
    if (event.persisted) show(stored());
  });
})();
"""


def map_link(href, label, current, number='', extra=''):
    """One entry in a side map, marked when it is the page being read."""
    here = ' aria-current="page"' if href == current else ''
    num = f'<span class="n">{number}</span>' if number != '' else ''
    return f'<a href="{href}"{here}>{num}<span>{label}{extra}</span></a>'


def planned_mark(item):
    """In a side map a planned page is greyed; a screen reader is told in words."""
    return '<span class="vh"> (planned)</span>' if item.get('status') == 'planned' else ''


def fold(label, items, here, number=''):
    """A folding group in a side map: open, and marked, if this page is in it."""
    num = f'<span class="sn">{number}</span>' if number != '' else ''
    opened, marked = (' open', ' class="here"') if here else ('', '')
    return (f'<details{opened}><summary{marked}>{num}<span>{label}</span></summary>'
            f'{items}</details>')


def stages_nav(view, current):
    """
    Every stage of one robot's course, folding; the one holding `current`
    is open with that page marked. Lessons not yet written for the robot
    are greyed.
    """
    parts = ['<p class="map-title">Stages</p>']
    for stage in view:
        overview = f'stage-{stage["id"]}.html'
        project = stage['project']
        mine = {overview, f'project-{stage["id"]}.html'}
        items = [f'<li>{map_link(overview, "Overview", current)}</li>']
        for lesson in stage['lessons']:
            href = f'lesson-{lesson["id"]}.html'
            mine.add(href)
            planned = ' class="planned"' if lesson.get('status') == 'planned' else ''
            items.append(f'<li{planned}>' + map_link(
                href, esc(lesson['title']), current, lesson['position'],
                planned_mark(lesson)) + '</li>')
        kind = 'proj planned' if project.get('status') == 'planned' else 'proj'
        items.append(f'<li class="{kind}">' + map_link(
            f'project-{stage["id"]}.html', 'Project: ' + esc(project['title']),
            current, '&#9670;', planned_mark(project)) + '</li>')
        parts.append(fold(esc(stage['title']), '<ol>' + ''.join(items) + '</ol>',
                          current in mine, stage_mark(stage)))
    return ''.join(parts)


def course_nav(robot, view, current, ready_total):
    """
    The course map down the side of every page of one robot's course.

    Every stage folds; the one this page belongs to is open, with this
    page marked, so a reader always sees where they are and what is
    either side. Lessons not yet written for this robot say so.
    """
    ready, total = ready_total
    home = ' aria-current="page"' if current == 'index.html' else ''
    parts = [f'<a class="home" href="index.html"{home}>The course on the '
             f'{esc(robot["name"])}</a>']
    if ready < total:
        parts.append(f'<p class="sub">{ready} of {total} lessons written for it</p>'
                     f'<div class="meter" aria-hidden="true"><span style="width:'
                     f'{round(100 * ready / total)}%"></span></div>'
                     f'<p class="sub">Greyed: planned, not written yet</p>')
    else:
        path = sum(not is_track(st) for st in view)
        parts.append(f'<p class="sub">{path} stages &middot; {total} lessons</p>')

    parts.append(stages_nav(view, current))

    parts.append('<p class="map-title">Look things up</p><ul class="flat">')
    for href, label in (('tags/index.html', 'Topics'), ('glossary.html', 'Glossary'),
                        ('robot.html', f'About the {esc(robot["name"])}'),
                        ('../reference.html', 'Reference docs')):
        link = map_link(href, label, current)
        if href.startswith('tags/') and current.startswith('tags/') and current != href:
            link = link.replace('<a ', '<a aria-current="true" ', 1)
        parts.append(f'<li>{link}</li>')
    parts.append('</ul>')
    return ''.join(parts)


def chooser_nav(robots, counts, ref, ref_view):
    """
    The map on the front page, before a robot is chosen: the same map as
    inside a course, its top box a choice of robot instead of the robot.

    A lesson needs a robot to open on. Until one is chosen it is the
    reference robot, whose text every lesson's is by default -- and the
    map says so, so nobody lands on the Arm 1 by surprise.
    """
    parts = ['<a class="home" href="index.html" aria-current="page">Choose your robot</a>',
             '<p class="sub">The same course on either arm</p><ul class="pick">']
    for robot in robots:
        ready, total = counts[robot['id']]
        parts.append(
            f'<li><a href="{robot["id"]}/index.html"><b>{esc(robot["name"])}</b>'
            f'<span>{ready} of {total} lessons &middot; {esc(robot["price"])}</span>'
            f'<span class="meter" aria-hidden="true"><span style="width:'
            f'{round(100 * ready / total) if total else 0}%"></span></span></a></li>')
    parts.append(f'</ul><p class="sub">Until you choose, lessons open on the '
                 f'{esc(ref["name"])}.</p>')
    parts.append(rebase(stages_nav(ref_view, ''), f'{ref["id"]}/'))
    parts.append('<p class="map-title">Look things up</p><ul class="flat">'
                 f'<li><a href="{ref["id"]}/tags/index.html">Topics</a></li>'
                 f'<li><a href="{ref["id"]}/glossary.html">Glossary</a></li>'
                 '<li><a href="reference.html">Reference docs</a></li></ul>')
    return ''.join(parts)


def reference_groups(docs, written):
    """
    The reference in reading order, as (heading, [(doc, label)]) groups:
    CURRICULUM's groups, then any doc it does not name -- so adding a doc
    never means it silently fails to appear -- then the decisions.
    """
    listed = {name for _, items in CURRICULUM for name, _ in items}
    groups = [(heading, [(n, label) for n, label in items if n in written])
              for heading, items in CURRICULUM]
    groups.append(('Everything else', [(n, doc_label(n))
                                       for n in sorted(set(written) - listed)]))
    groups.append(('Decisions', [('decisions/' + a.name, doc_label('decisions/' + a.name))
                                 for a in sorted((docs / 'decisions').glob('*.md'))]))
    return [(heading, items) for heading, items in groups if items]


def reference_nav(groups, robots, current):
    """The map down the side of a reference page, and the way back to a course."""
    home = ' aria-current="page"' if current == 'reference.html' else ''
    parts = [f'<a class="home" href="reference.html"{home}>Reference</a>',
             '<p class="sub">The project\'s own documentation</p>',
             '<p class="map-title">Sections</p>']
    for heading, items in groups:
        links = ''.join(f'<li>{map_link(doc_page(n), esc(label), current)}</li>'
                        for n, label in items)
        here = current in {doc_page(n) for n, _ in items}
        parts.append(fold(esc(heading), f'<ul>{links}</ul>', here))
    parts.append('<p class="map-title">The course</p><ul class="flat">')
    parts.extend(f'<li><a href="{r["id"]}/index.html">On the {esc(r["name"])}</a></li>'
                 for r in robots)
    parts.append('<li><a href="index.html">Compare the robots</a></li></ul>')
    return ''.join(parts)


def doc_frame(order, position, html):
    """
    A reference page with where it sits above it and its neighbours below,
    in the same reading order as the map, the way a lesson has them.
    """
    name, _, heading = order[position]

    def link(pos, left):
        other, label, _ = order[pos]
        word = '&larr; Previous' if left else 'Next &rarr;'
        return (f'<a class="{"prev" if left else "next"}" href="{doc_page(other)}">'
                f'<small>{word}</small>{esc(label)}</a>')

    prev = link(position - 1, True) if position else '<span></span>'
    nxt = (link(position + 1, False) if position + 1 < len(order)
           else '<a class="next" href="reference.html"><small>That is all</small>'
                'Back to the reference</a>')
    return (f'<div class="crumbs"><a href="reference.html">Reference</a> / '
            f'{esc(heading)}</div>\n{html}\n<div class="pager">{prev}{nxt}</div>')


def text_of(html):
    """The words of an HTML fragment, for a label: tags dropped, spaces kept."""
    text = re.sub(r'<[^>]+>', '', html.replace('</span>', '</span> '))
    return ' '.join(htmllib.unescape(text).split())


def outline(html, least=3):
    """
    Give every <h2> an id, and list them as the page's "On this page".

    Docs arrive with ids from markdown's toc extension; lesson sections
    are written here without, so they get one from their words, never
    one the page already uses. A page with fewer than `least` sections
    gets no outline: it would only repeat the page.
    """
    taken = set(re.findall(r'\sid="([^"]+)"', html))
    found = []

    def mark(match):
        attrs, inner = match.group(1) or '', match.group(2)
        has = re.search(r'\sid="([^"]+)"', attrs)
        if has:
            anchor = has.group(1)
        else:
            base = slug(text_of(inner)) or 'section'
            anchor, n = base, 2
            while anchor in taken:
                anchor, n = f'{base}-{n}', n + 1
            taken.add(anchor)
            attrs += f' id="{anchor}"'
        found.append((anchor, text_of(inner)))
        return f'<h2{attrs}>{inner}</h2>'

    html = re.sub(r'<h2(\s[^>]*)?>(.*?)</h2>', mark, html, flags=re.S)
    if len(found) < least:
        return html, ''
    items = ''.join(f'<li><a href="#{a}">{esc(t)}</a></li>' for a, t in found)
    return html, (f'<nav class="toc" aria-label="On this page"><b>On this page</b>'
                  f'<ul>{items}</ul></nav>')


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
    dots = ''.join('<i class="here"></i>' if n == lesson['position'] else
                   '<i class="past"></i>' if n < lesson['position'] else '<i></i>'
                   for n in range(1, count + 1))
    parts = [
        crumbs(stage),
        f'<div class="lesson-head">{stage_name(stage)} '
        f'&middot; lesson {lesson["position"]} of {count} &middot; '
        f'{hours(lesson["minutes"])}<span class="dots" aria-hidden="true">'
        f'{dots}</span></div>',
        f'<h1>{esc(lesson["title"])}</h1>',
        f'<p class="objective"><b>You will:</b> {inline(lesson["objective"])}</p>',
        tag_chips(lesson['tags'], vocab),
    ]
    # A lesson built on may not exist on this robot (it needs wheels).
    deps = [by_id[dep] for dep in lesson.get('builds_on') or [] if dep in by_id]
    if deps:
        parts.append('<p class="builds">Builds on '
                     + said(lesson_link(dep) for dep in deps) + '.</p>')
    touched = []
    for mention in lesson.get('mentions') or []:
        if mention['tag'] not in touched:
            touched.append(mention['tag'])
    if touched:
        parts.append('<p class="builds">Also touches on '
                     + said(f'<a href="{tag_href(t)}">{esc(vocab[t]["label"])}</a>'
                            for t in touched) + '.</p>')
    parts.append(rewrite_links(render(lesson.get('body', ''))))
    parts.extend(robot_notes(lesson, f'lesson-{lesson["id"]}.html'))

    if lesson.get('status') == 'planned':
        # The idea is shared and real; the steps, checks and pitfalls
        # are the reference robot's, and would mislead on this one.
        if lesson.get('concepts'):
            parts.append('<h2>Terms</h2><dl class="terms">')
            parts.extend(f'<dt id="{slug(c["term"])}">{esc(c["term"])}</dt>'
                         f'<dd>{inline(c["meaning"])}</dd>' for c in lesson['concepts'])
            parts.append('</dl>')
        parts.append(pager(sequence, position))
        return '\n'.join(parts)

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
        f'<div class="lesson-head">{stage_name(stage)} '
        f'&middot; project &middot; {hours(project["minutes"])}</div>',
        f'<h1>Project: {esc(project["title"])}</h1>',
        rewrite_links(render(project['brief'])),
    ]
    parts.extend(robot_notes(project, f'project-{stage["id"]}.html'))
    if project.get('status') == 'planned':
        parts.append(pager(sequence, position))
        return '\n'.join(parts)
    parts.append('<h2>Tasks</h2><ol class="steps">')
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
        f'<div class="lesson-head">{stage_name(stage)} '
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
            f'{esc(lesson["title"])}</a>{soon(lesson)}<span class="obj">'
            f'{inline(lesson["objective"])}</span>{tag_chips(lesson["tags"], vocab)}</td>'
            f'<td class="m">{lesson["minutes"]} min</td></tr>')
    project = stage['project']
    parts.append(
        f'<tr><td>&#9670;</td><td><a href="project-{stage["id"]}.html">Project: '
        f'{esc(project["title"])}</a>{soon(project)}</td>'
        f'<td class="m">{project["minutes"]} min</td></tr>')
    parts.append('</table>')

    robot, ref = stage.get('robot'), stage.get('ref')
    if robot and stage.get('skipped'):
        parts.append(f'<h2>Not on the {esc(robot["name"])}</h2><ul>')
        for lesson, missing in stage['skipped']:
            parts.append(
                f'<li>{esc(lesson["title"])}<span class="obj">needs '
                f'{" and ".join(CAPABILITIES[c] for c in missing)}, which this '
                f'robot does not have. '
                f'<a href="../{ref["id"]}/lesson-{lesson["id"]}.html">Read it on the '
                f'{esc(ref["name"])}</a></span></li>')
        parts.append('</ul>')

    first = (f'lesson-{lessons[0]["id"]}.html' if lessons
             else f'project-{stage["id"]}.html')
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
            f'<a href="lesson-{lesson["id"]}.html">{stage_name(stage)}, '
            f'lesson {lesson["position"]}</a></dd>')
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
            f'<h2 class="stage-head"><span class="n">{stage_name(stage)}'
            f'</span><a href="stage-{stage["id"]}.html">'
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
            f'<li><a href="lesson-{le["id"]}.html">{esc(le["title"])}</a>{soon(le)}'
            f'<span class="mins">{le["minutes"]} min</span></li>'
            for le in stage['lessons'])
        project = stage['project']
        items += (f'<li class="proj"><a href="project-{stage["id"]}.html">Project: '
                  f'{esc(project["title"])}</a>{soon(project)}</li>')
        skipped = (f' &middot; {len(stage["skipped"])} not on this robot'
                   if stage.get('skipped') else '')
        out.append(
            f'<section class="stage">'
            f'<div class="num" aria-hidden="true">{stage_mark(stage, 2)}</div>'
            f'<div class="card"><header>'
            f'<span class="n">{stage_name(stage)}</span>'
            f'<h2><a href="stage-{stage["id"]}.html">{esc(stage["title"])}</a></h2>'
            f'{price_badge(stage)}</header>'
            f'<p class="blurb">{inline(stage["summary"])}</p>'
            f'<div class="facts">{len(stage["lessons"])} lessons &middot; '
            f'project &middot; {hours(stage["minutes"])}{skipped}</div>'
            f'<ol>{items}</ol></div></section>')
    return '<div class="ladder">' + '\n'.join(out) + '</div>'


# Every lesson has the same parts in the same order, so a student always
# knows where to look. Shown as tiles on the front pages.
LESSON_PARTS = (
    ('Why', 'Why the thing is built this way, usually through something '
            'that went wrong here first.'),
    ('Do it', 'Numbered steps with the exact command and what you should see.'),
    ('Checkpoint', 'One sign that tells you it worked.'),
    ('When it goes wrong', 'The symptoms that mislead, and what they really mean.'),
    ('Check yourself', 'Questions with the answers folded away.'),
    ('Stretch', 'An exercise that goes past the steps.'),
)

HOW_IT_WORKS = (
    'Every stage ends in a **project** with a definition of done.\n\n'
    'You need Docker and a machine with 6 GB of memory to give it. You '
    'do not need a robot to start: hardware is one of three interchangeable '
    'targets, and the course is built so the other two teach '
    'everything except the screwdriver.')


def how_it_works():
    """The shape every lesson shares, then what you need to start."""
    tiles = ''.join(f'<li><b>{esc(name)}</b>{esc(line)}</li>'
                    for name, line in LESSON_PARTS)
    return (f'<h2>How every lesson works</h2>'
            f'<p>Each lesson has the same shape, so you always know where to look.</p>'
            f'<ol class="anatomy">{tiles}</ol>{render(HOW_IT_WORKS)}')


def index_html(stages, robot, ref, counts, vocab):
    """Render one robot's front page: its course, and how far it has got."""
    tracks = [st for st in stages if is_track(st)]
    stages = [st for st in stages if not is_track(st)]
    lessons = [le for st in stages for le in st['lessons']]
    total_minutes = sum(st['minutes'] for st in stages)
    free = [st for st in stages if st.get('price') == 'free']
    paid = [st for st in stages if st.get('price') != 'free']
    cost = (f"${sum(int(st['price'].strip('$')) for st in paid)}" if paid else '$0')
    ready, total = counts[robot['id']]
    is_ref = robot['id'] == ref['id']

    parts = [f'<section class="hero split">{robot_art(robot, "robot.html")}'
             f'<div class="eyebrow">The course on the {esc(robot["name"])}</div>'
             f'<h1>Build a robot the way you build software</h1>'
             f'<p class="lede">One system that grows, from a simulated robot to one '
             f'that sees and learns. You already know Git, Docker, testing, CI and '
             f'observability. This is those skills, pointed at a machine that '
             f'moves.</p></section>',
             f'<p>{inline(robot["summary"])} <a href="robot.html">About the '
             f'{esc(robot["name"])} &rarr;</a></p>']

    stats = [
        f'<div class="stat"><b>{len(lessons)}</b><span>lessons on this robot</span></div>',
        f'<div class="stat"><b>{len(stages)}</b><span>stages, each ending in a project</span></div>',
        f'<div class="stat"><b>~{round(total_minutes / 60)} h</b><span>of hands-on work</span></div>',
        (f'<div class="stat"><b>{len(free)} free</b><span>then '
         f'{paid[0]["price"]} per tier, {cost} for all</span></div>' if paid else
         f'<div class="stat"><b>Free</b><span>all {len(stages)} stages, '
         f'for now</span></div>'),
    ]
    if not is_ref:
        stats.insert(1, f'<div class="stat"><b>{ready} of {total}</b>'
                        f'<span>written for this robot</span></div>')
    parts.append('<div class="stats">' + ''.join(stats) + '</div>')

    if is_ref:
        parts.append(
            '<div class="note"><strong>This is a live preview of an '
            'unfinished course.</strong> It is generated from the project as '
            'it actually stands, including what does not work yet. Where '
            'something is unproven or failing, the lesson says so.</div>')
    else:
        parts.append(
            f'<div class="note"><strong>The {esc(robot["name"])} version is being '
            f'written: {ready} of {total} lessons are ready.</strong> The rest '
            f'show what they will teach, marked planned, and link to the same '
            f'lesson on the {esc(ref["name"])} in simulation, which needs no '
            f'hardware.</div>')

    parts.append(how_it_works())
    parts.append('<h2>The course</h2>')
    parts.append(ladder_html(stages))
    if tracks:
        parts.append('<h2>Side tracks</h2>')
        parts.append(render(
            'Off the numbered path, for when you want to go further in one '
            'direction. Take one after the stage it builds on, or after the '
            'whole course. They are not counted in the lessons above.'))
        parts.append(ladder_html(tracks))

    tag_counts = {name: 0 for name in vocab}
    for le in lessons:
        for tag in le['tags']:
            tag_counts[tag] += 1
    shown = [name for name in vocab if tag_counts[name]]
    parts.append('<h2>Topics</h2>')
    parts.append(render(
        'Every lesson is tagged. Pick a topic to see each lesson that covers '
        'it, wherever it sits in the course, and what to read alongside.'))
    parts.append(tag_chips(shown, vocab, counts=tag_counts))
    parts.append('<p><a href="tags/index.html">Every topic, described &rarr;</a></p>')
    parts.append('<p><a href="glossary.html">Glossary of every term the course '
                 'introduces &rarr;</a></p>')
    parts.append('<p><a href="../reference.html">The project\'s reference '
                 'documentation &rarr;</a></p>')
    return '\n'.join(parts)


def robot_page(robot, stages, ref, counts):
    """One robot: what it costs, what only it teaches, and how far its course is."""
    ready, total = counts[robot['id']]
    parts = ['<div class="crumbs"><a href="index.html">The course</a></div>',
             f'<section class="hero split">{robot_art(robot)}'
             f'<div class="eyebrow">{esc(robot["kind"])}</div>',
             f'<h1>{esc(robot["name"])} <span class="price">{esc(robot["price"])}</span></h1>'
             f'</section>',
             rewrite_links(render(robot['summary'])),
             '<h2>What it costs</h2>', rewrite_links(render(robot['price_note'])),
             '<h2>What it teaches that the others do not</h2><ul>']
    parts.extend(f'<li>{inline(t)}</li>' for t in robot['teaches'])
    parts.append('</ul>')
    if robot.get('exclusive'):
        parts.append('<h2>Lessons only this robot gets</h2>'
                     '<p class="expect">Planned, not written yet.</p><ul>')
        parts.extend(f'<li>{inline(t)}</li>' for t in robot['exclusive'])
        parts.append('</ul>')
    parts.append('<h2>Where it stands</h2>' + rewrite_links(render(robot['status'])))

    parts.append('<h2>The course on this robot</h2><table class="syllabus"><tr>'
                 '<th>Stage</th><th>Ready</th><th>Planned</th><th>Not on this robot</th></tr>')
    def row(st):
        done = sum(le['status'] == 'ready' for le in st['lessons'])
        return (f'<tr><td><a href="stage-{st["id"]}.html">{stage_name(st)}: '
                f'{esc(st["title"])}</a></td><td class="m">{done}</td>'
                f'<td class="m">{len(st["lessons"]) - done}</td>'
                f'<td class="m">{len(st.get("skipped") or [])}</td></tr>')

    path = [st for st in stages if not is_track(st)]
    parts.extend(row(st) for st in path)
    parts.append(f'<tr><th>All</th><th>{ready}</th><th>{total - ready}</th>'
                 f'<th>{sum(len(st.get("skipped") or []) for st in path)}</th></tr>')
    # Side tracks after the total, which they are not part of.
    parts.extend(row(st) for st in stages if is_track(st))
    parts.append('</table>')
    if robot.get('read'):
        parts.append('<h2>Going deeper</h2>' + read_list(robot['read']))
    return '\n'.join(parts)


REFERENCE_INTRO = ('The project\'s own documentation, linked from the lessons that '
                   'need it. You do not have to read it in order.')


def reference_list(groups):
    """The reference's groups as lists of links."""
    parts = []
    for heading, items in groups:
        rows = ''.join(f'<li><a href="{doc_page(n)}">{esc(label)}</a></li>'
                       for n, label in items)
        parts.append(f'<div class="group"><h3>{esc(heading)}</h3><ul>{rows}</ul></div>')
    return '\n'.join(parts)


def reference_html(groups):
    """The reference as a section of the front page."""
    return '\n'.join(['<h2 id="reference">Reference</h2>', render(REFERENCE_INTRO),
                      reference_list(groups)])


def reference_page(groups):
    """The reference's own index page, the Reference tab."""
    count = sum(len(items) for _, items in groups)
    return '\n'.join([
        '<div class="eyebrow">Look it up</div><h1>Reference</h1>',
        render(f'{REFERENCE_INTRO} {count} pages in {len(groups)} sections; '
               'the map at the side follows you through them.'),
        reference_list(groups)])


def chooser_html(robots, counts, docs, groups):
    """
    The front page: one course, and the robots it can be taken on.

    Each card says what the robot costs, what only it teaches, and how
    much of the course is actually written for it -- counted, not claimed.
    """
    parts = ['<section class="hero">'
             '<div class="eyebrow">A hands-on course in robot software</div>'
             '<h1>One course. Pick your robot.</h1>'
             '<p class="lede">The same engineering — tests, CI, deployment, safety, '
             'perception — taught on two inexpensive arms: the 3VN Arm 1, which you '
             'print and wire yourself, and the SO-101, an open arm whose servos '
             'report back. Every lesson\'s idea is shared; its steps are written for '
             'your robot. Start free on either: simulation needs no hardware.</p>'
             '</section>', '<div class="robots">']

    for robot in robots:
        ready, total = counts[robot['id']]
        teach = ''.join(f'<li>{inline(t)}</li>' for t in robot['teaches'])
        course = f'{robot["id"]}/index.html'
        parts.append(
            f'<article class="robot">{robot_art(robot, course)}<div class="body">'
            f'<div class="eyebrow">{esc(robot["kind"])}</div>'
            f'<h2><a href="{course}">{esc(robot["name"])}</a></h2>'
            f'<span class="price">{esc(robot["price"])}</span>'
            f'<p>{inline(robot["summary"])}</p>'
            f'<div class="meter" title="{ready} of {total} lessons">'
            f'<span style="width:{round(100 * ready / total) if total else 0}%"></span></div>'
            f'<div class="stage-facts expect">{ready} of {total} lessons written for it</div>'
            f'<ul class="teach">{teach}</ul>'
            f'<p class="go"><a class="button" href="{course}">Start on the '
            f'{esc(robot["name"])} &rarr;</a>'
            f'<a href="{robot["id"]}/robot.html">About it</a></p></div></article>')
    parts.append('</div>')

    parts.append(how_it_works())

    table = phase_table(docs)
    if table:
        parts.append('<h2>The real project, honestly</h2>')
        parts.append(render(
            'The course is written against a project that is genuinely '
            'being built, not a finished example. This is where it is:'))
        parts.append(render(table))

    parts.append(reference_html(groups))
    return '\n'.join(parts)


# ------------------------------------------------------------------ build

REDIRECT = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="robots" content="noindex, nofollow">
<meta http-equiv="refresh" content="0; url={target}">
<title>Moved — 3VN Robotics</title></head>
<body><p><a href="{target}">This page has moved.</a></p></body></html>
"""


def relink_docs(html, doc_pages):
    """
    Point a robot's pages at the shared docs one directory up.

    Lessons live in site/<robot>/ and link docs as `viewing.html`, which
    is right at the top level and wrong one level down. Only names that
    ARE doc pages move, so lesson-to-lesson links stay where they are.
    """
    def fix(match):
        name, anchor = match.group(1), match.group(2) or ''
        return f'href="../{name}{anchor}"' if name in doc_pages else match.group(0)
    return re.sub(r'href="([^"/:#]+\.html)(#[^"]*)?"', fix, html)


def build(root, out, stages, robots, tags):
    """Render the whole site into `out`. Returns the page count."""
    docs = root / 'docs'
    out.mkdir(parents=True, exist_ok=True)

    version = (root / 'VERSION').read_text().strip() if (root / 'VERSION').exists() else ''
    # git is absent inside the build image, so the commit is passed in.
    commit = (run(['git', 'rev-parse', '--short', 'HEAD'], root)
              or os.environ.get('THREEVN_GIT_COMMIT', '')[:7])
    meta = f'v{version} · {commit or "no commit"} · built {datetime.date.today().isoformat()}'

    def page(title, body, home='index.html', picker='', tabs='', side='', label='',
             toc=''):
        # A page with a side map is framed: map, page, and -- when the page
        # is long enough to have one -- its outline. Without one (the front
        # page) it is the page alone, at reading width.
        toggle = menu = wide = ''
        if side:
            body = (f'<div class="frame{" has-toc" if toc else ""}">'
                    f'<aside class="side" aria-label="{label}">{side}</aside>'
                    f'<main>{body}</main>{toc}</div>')
            # autocomplete=off: or Back would restore it ticked, the map open.
            toggle = '<input type="checkbox" id="menu" class="vh" autocomplete="off">'
            menu = ('<label class="menu-btn" for="menu"><i aria-hidden="true"></i>'
                    '<span class="open">Menu</span><span class="shut">Close</span></label>')
            wide = ' wide'
        else:
            body = f'<main>{body}</main>'
        # nav.js and theme.js sit at the site root, beside the front page
        # `home` names.
        script = home[:-len('index.html')] + 'nav.js'
        theme_js = home[:-len('index.html')] + 'theme.js'
        return PAGE.format(title=esc(title), style=STYLE, meta=meta, body=body,
                           home=home, picker=picker, mark=MARK, tabs=tabs,
                           toggle=toggle, menu=menu, wide=wide, script=script,
                           theme_js=theme_js, theme=THEME_SWITCH)

    written = [source.name for source in sorted(docs.glob('*.md'))]
    groups = reference_groups(docs, written)
    order = [(name, label, heading) for heading, items in groups for name, label in items]

    def shared(name, title, body, side=None, active='reference', label='Reference'):
        # The pages every robot shares, at the top level: the reference and
        # the front page. A reference page's map is the reference's.
        body, toc = outline(body)
        (out / name).write_text(page(
            title, body, tabs=tabs_html(active, 'index.html', 'reference.html'),
            side=side or reference_nav(groups, robots, name), label=label, toc=toc))

    for position, (name, label, _) in enumerate(order):
        html = rewrite_links(render((docs / name).read_text()))
        shared(doc_page(name), label, doc_frame(order, position, html))
    shared('reference.html', 'Reference', reference_page(groups))
    doc_pages = {p.name for p in out.glob('*.html')}

    ref = reference_of(robots)
    views = {r['id']: flavor_view(stages, r, ref) for r in robots}
    counts = {rid: coverage(view) for rid, view in views.items()}
    vocab = {tag['slug']: tag for tag in tags}

    def sequence_of(view):
        sequence = []
        for stage in view:
            sequence.extend(('lesson', le) for le in stage['lessons'])
            sequence.append(('project', stage['project']))
        return sequence

    pages = {}
    for rid, view in views.items():
        names = {'index.html', 'robot.html', 'glossary.html', 'tags/index.html'}
        for stage in view:
            names.add(f'stage-{stage["id"]}.html')
            names.add(f'project-{stage["id"]}.html')
            names.update(f'lesson-{le["id"]}.html' for le in stage['lessons'])
        names.update(tag_href(tag['slug']) for tag in tags)
        pages[rid] = names

    for robot in robots:
        view = views[robot['id']]
        folder = out / robot['id']
        folder.mkdir(exist_ok=True)
        by_id = {le['id']: le for st in view for le in st['lessons']}

        def fwrite(name, title, body, robot=robot, folder=folder, view=view):
            # Topic pages sit in tags/<slug>/ inside the robot's folder. Doc
            # links are pointed up at the shared docs first, then the whole
            # page is re-rooted for its depth.
            prefix = '../' * name.count('/')
            body, toc = outline(body)
            html = page(f'{title} · {robot["name"]}',
                        rebase(relink_docs(body, doc_pages), prefix),
                        home=prefix + '../index.html',
                        picker=rebase(picker_html(robot, robots, pages, counts, name),
                                      prefix),
                        tabs=rebase(tabs_html('course', 'index.html', '../reference.html'),
                                    prefix),
                        side=rebase(course_nav(robot, view, name, counts[robot['id']]),
                                    prefix),
                        label='Course map', toc=toc)
            path = folder / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(html)

        sequence = sequence_of(view)
        for position, (kind, obj) in enumerate(sequence):
            if kind == 'lesson':
                fwrite(f'lesson-{obj["id"]}.html', obj['title'],
                       lesson_page(obj, sequence, position, vocab, by_id))
            else:
                fwrite(f'project-{obj["stage"]["id"]}.html', f'Project: {obj["title"]}',
                       project_page(obj['stage'], sequence, position))
        for stage in view:
            fwrite(f'stage-{stage["id"]}.html', stage['title'],
                   stage_page(stage, sequence, vocab))
        for tag in tags:
            fwrite(tag_href(tag['slug']), f'Topic: {tag["label"]}',
                   tag_page(tag, view, vocab))
        fwrite('tags/index.html', 'Topics', tags_index(view, vocab))
        fwrite('glossary.html', 'Glossary', glossary_page(view))
        fwrite('robot.html', robot['name'], robot_page(robot, view, ref, counts))
        fwrite('index.html', 'The course', index_html(view, robot, ref, counts, vocab))

    # The course used to live at the top level, its topic pages under
    # tags/. Keep those addresses working: a refresh needs no script, so
    # the CSP allows it.
    for name in sorted(pages[ref['id']] - {'index.html', 'robot.html'} - doc_pages):
        path = out / name
        path.parent.mkdir(parents=True, exist_ok=True)
        up = '../' * name.count('/')
        path.write_text(REDIRECT.format(target=f'{up}{ref["id"]}/{name}'))

    shared('index.html', 'Pick your robot', chooser_html(robots, counts, docs, groups),
           side=chooser_nav(robots, counts, ref, views[ref['id']]),
           active='course', label='Course map')
    (out / 'robots.txt').write_text('User-agent: *\nDisallow: /\n')
    (out / 'nav.js').write_text(NAV_JS)
    (out / 'theme.js').write_text(THEME_JS)
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
    robots = load_robots(root)
    errors = validate(stages, root, tags, robots)
    if errors:
        print(f'  course/ has {len(errors)} problem(s):', file=sys.stderr)
        for error in errors:
            print(f'    {error}', file=sys.stderr)
        return 1

    path = [st for st in stages if not is_track(st)]
    tracks = [st for st in stages if is_track(st)]
    lessons = sum(len(st['lessons']) for st in path)
    minutes = sum(st['minutes'] for st in path)
    print(f'  course ok: {len(path)} stages, {lessons} lessons, '
          f'{len(path)} projects, {len(tags)} topics, {hours(minutes)}')
    for track in tracks:
        print(f'    side track {track["id"]}: {len(track["lessons"])} lessons '
              f'and a project, {hours(track["minutes"])}')
    ref = reference_of(robots)
    for robot in robots:
        ready, total = coverage(flavor_view(stages, robot, ref))
        print(f'    {robot["id"]:<10} {ready:>2} of {total} lessons written for it')
    if args.check:
        return 0

    out = pathlib.Path(args.out).resolve()
    if args.clean and out.exists():
        shutil.rmtree(out)
    count = build(root, out, stages, robots, tags)
    print(f'  {count} pages -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

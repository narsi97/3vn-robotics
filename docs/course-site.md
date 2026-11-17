# The course site

```bash
make site          # render docs/ into site/
make site-serve    # and look at it: http://localhost:8109/
```

## What it is

A course, not a manual. Six stages, all free for now. Stages 4–6 are
tiers, planned at $5 each once the course matures; their YAML records
that next to `price: free`, and putting the price back is all it takes
to show it again. Every stage opens with what you will be able to do by
the end, and closes with a **project** that has a definition of done.

Every lesson has the same shape, so a student learns where to look:

| part | YAML key | what it is |
|---|---|---|
| objective | `objective` | one sentence: what you will be able to do |
| why | `body` | the reasoning, usually through something that went wrong here first |
| do it | `steps` | numbered; each a `do`, an optional `run`, an optional `expect` |
| checkpoint | `checkpoint` | how you know it worked |
| when it goes wrong | `pitfalls` | `symptom` → `cause` → `fix` |
| terms | `concepts` | `term` + `meaning`; collected into the glossary page |
| check yourself | `quiz` | `q` + `a`; answers fold away in `<details>`, which needs no script |
| stretch | `exercise` | a `task`, with an optional `hint` and `answer` |
| going deeper | `read` | docs in `docs/`, linked by their rendered page |

Plus `minutes`, so stage and course durations are **summed, never
typed** — the same rule as the robot's inertia tensors.

The curriculum is DATA, in `course/*.yaml`. Adding a lesson is adding a
block to a YAML file; the generator knows nothing about any particular
lesson. That is the same reasoning that puts the robot's dimensions in
YAML rather than in the xacro.

### It is validated before it is rendered

```bash
make course-check
```

A lesson that tells a student to run `make foo` when the Makefile has
no `foo`, or links a doc that has been renamed, is a broken lesson that
looks finished. So before rendering, the generator checks:

- every stage, lesson and project has the fields its page renders
- every `make <target>` in a step, a pitfall's fix or inline code exists
  in the Makefile — prose is not checked, only code
- every doc a lesson links or reads exists in `docs/`
- lesson ids and glossary terms are unique, and stages are numbered in
  file order

Every problem is reported at once, not the first. A failure stops
`make site`, the CI `course` job, and the deploy build — so a broken
curriculum never replaces the one already being served. The checker was
mutation-tested when it was written: four planted faults, four named
errors.

## One course, two robots

The course is taken on one of two arms: the 3VN Arm 1 and the SO-101.
Other robots wait until there is a clear plan for them, and recordings
for each. A student picks the robot they learn on, and the site is
built once per robot, under `site/<robot>/`, topic pages included. The front page compares them; a
dropdown at the top of every course page switches robot and lands on
the same lesson where that robot has it. It is a `<details>`, so it
needs no JavaScript.

The robots are data too, one file each in `course/robots/`: name,
price, what it `has` (`arm`, `gripper`, `wheels`, `legs`, `camera`,
`lidar`), what only it teaches, and where it stands. Exactly one is
`reference: true`, the 3VN Arm 1, and every lesson's default text is
that robot's version. The Arm 1 has the optional mobile base, so the
lessons that need `wheels` are not on the SO-101.

The idea is shared and the steps are not. A lesson is one of three
kinds:

| in the lesson's YAML | means | on another robot |
|---|---|---|
| `scope: any` | robot-agnostic: Docker, CI, releases | the same page |
| `needs: [arm]` | only robots that have it | left out, and listed as "not on this robot" with the reason |
| neither | written for the reference robot | its own version if it has one, otherwise **planned** |

A robot's own version goes under the lesson:

```yaml
  - id: links-and-joints
    ...
    flavors:
      so101:
        steps: [...]      # required: without steps it stays planned
        checkpoint: ...
        pitfalls: [...]
        note: ...         # shown as "On the SO-101: ..."
        concepts: [...]   # added to the lesson's terms, not swapped
```

Only steps, checkpoint, pitfalls, quiz, exercise, read, note, minutes
and concepts can differ by robot. Projects work the same way, with
`tasks` required. A lesson only one robot gets uses
`only: [so101]`.

A planned page shows the shared objective, reasoning and terms, says
plainly that its steps are not written, and links to the same lesson
on the reference robot in simulation. Its steps, checks and pitfalls
are not shown: they are the reference robot's, and would mislead on
another one.

**Coverage is counted, never claimed.** `make course-check` prints
how many lessons are ready on each robot, and each robot's page has the
same count per stage. A robot's version is held to exactly the checks a
lesson is, so an SO-101 step that runs a Make target that does not exist
fails the build like any other. The checker was mutation-tested when it
was first written, for five robots: nine planted faults, nine named
errors.

The course used to live at the top level, and its topic pages under
`tags/`. Those addresses now redirect to the reference robot's copy
with a meta refresh, which the CSP allows.

The project's documentation is still rendered, underneath, as reference
that lessons link into. The course is the front page and the docs are
the appendix, rather than the other way round.

Tiers are **shown, not enforced**. The lock badge and the price are
visible; nothing checks them. There are no accounts and no payment, so
anyone with the link reads everything.

It also publishes the parts that do not work. The index says so in a
banner, and the grasp lesson ends on an open problem. A course that only
shows the finished half teaches the wrong thing about engineering.

## Three constraints, all from where it is served

It sits behind the shared 3VNSYSTEMS Caddy, at a public path,
`3vnsystems.com/robotics/`, which dictates more than it looks like:

- **Every link is relative.** The site lives under `/robotics/` today,
  an older unlisted path still serves it for links shared before it went
  public, and it may live at a bare domain tomorrow. Per
  `projects-for-businesses/README.md`, each move is meant to be a config
  change rather than a rewrite, so nothing may hardcode the prefix.
- **No JavaScript at all.** The enforced CSP is
  `script-src 'self'` with no `'unsafe-inline'`, so an inline `<script>`
  would be blocked. Styles are inline because `style-src` does permit
  `'unsafe-inline'`. A reading site needs no script anyway.
- **noindex everywhere**, plus `robots.txt` with `Disallow: /`, plus an
  `X-Robots-Tag` header from nginx. One of the three may be ignored by a
  given crawler; all three being ignored is unlikely.

**Public, but not yet indexed.** Since 2026-11-15 the course and its
repository are public and free. The noindex keeps an unfinished course
out of search results until it is ready to be found; it hides nothing,
and nothing on the site is secret.

## Deploying it

The image is built and deployed from **3vn-infra**, not from here:
Actions → **Course site** → Run workflow in that repo
(`.github/workflows/robotics-course.yml`), with a commit from this
repo's main (blank means the tip of main).

That run checks this repo out, takes the newest commit at or before the
one given that changed the site, and runs the same stages as every
other product: scan, build `docker/Dockerfile.site`, push
`ghcr.io/narsi97/robotics-course-site:sha-<commit>`, then deploy it to
the shared host, health-checked and rolled back if it isn't healthy. A
commit already in the registry is deployed as is, not rebuilt. **Going
back** is the same button with the previous good commit.

Why there: this repository is public, and GitHub lets a private repo's
reusable workflows be called only from private repos, so this repo
can't call 3vn-infra's pipeline. It also means nothing in this public
repo can reach a host. What runs here
(`.github/workflows/course-site.yml`) is the check on every PR and
merge that touches the site: `make course-check`, `make site`, and a
`docker build` of the image that is never pushed. No secrets, so fork
PRs are safe to run.

The service (`robotics-course-site` in 3vn-infra's
`docker/compose.prod.yml`) and its Caddy route live in 3vn-infra too.

## What is NOT served

The simulation. Gazebo needs far more than this box has — 1 vCPU and
2 GB against a simulator that wants 6 GB and four cores — and
`docs/learning-path.md` is honest that the simulation runs on your own
machine. The site is the reading half of the course. The doing half
stays local, which is also where the robot is.

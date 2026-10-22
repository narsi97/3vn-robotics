# The course site

```bash
make site          # render docs/ into site/
make site-serve    # and look at it: http://localhost:8109/
```

## What it is

A course, not a manual. Six stages: the first three free, then three
tiers at $5 each. Every stage opens with what you will be able to do by
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

It sits behind the shared 3VNSYSTEMS Caddy on an unlisted path, which
dictates more than it looks like:

- **Every link is relative.** The site lives under a secret path today
  and may live at a bare domain tomorrow. Per
  `projects-for-businesses/README.md`, that move is meant to be a config
  change rather than a rewrite, so nothing may hardcode the prefix.
- **No JavaScript at all.** The enforced CSP is
  `script-src 'self'` with no `'unsafe-inline'`, so an inline `<script>`
  would be blocked. Styles are inline because `style-src` does permit
  `'unsafe-inline'`. A reading site needs no script anyway.
- **noindex everywhere**, plus `robots.txt` with `Disallow: /`, plus an
  `X-Robots-Tag` header from nginx. One of the three may be ignored by a
  given crawler; all three being ignored is unlikely.

**This is obscurity, not security.** Anyone given the link can pass it
on. It keeps an unfinished course out of search results; it is not a
login, and nothing behind it is secret.

## Deploying it

> The secret path is **not in this repository and must not be**. This
> repository is private today, but it was public and may be again, and
> anything committed stays in its history. Delivery Manager's path was
> kept out of git for the same reason — see
> `3vnsystems-infrastructure/ARCHITECTURE.md`.

That file also says, and it is load bearing:

> Never copy a local infra file over the VPS one; you will delete live
> config. Patch the VPS copies in place, and back them up first.

So this is a patch, applied on the box, by hand. Three edits.

**1. `docker/compose.prod.yml`** — add the service:

```yaml
  robotics-course:
    build:
      context: ${ROBOTICS_REPO_PATH}
      dockerfile: docker/Dockerfile.site
      args:
        THREEVN_GIT_COMMIT: "${ROBOTICS_GIT_COMMIT:-unknown}"
    image: threevn-robotics-course:local
    restart: unless-stopped
    expose:
      - "80"
```

A few MB of nginx. It carries no ROS: the dev image is several GB and
this box has 2 GB of RAM and about ten containers on it already, which
is exactly why the site is a separate image.

**2. `caddy/Caddyfile`** — add inside the `{$DOMAIN}` block, next to the
other products:

```
	# --- 3vn-robotics course (unlisted) ---
	handle_path /rc-<secret>/* {
		reverse_proxy robotics-course:80
	}
```

`handle_path` strips the prefix, which is what lets the site be built
with relative links and know nothing about where it is mounted.

**3. `deploy.sh`** — add the pull, alongside the others:

```sh
pull_if_tracked "${ROBOTICS_REPO_PATH}" "3vn-robotics"
```

and set `ROBOTICS_REPO_PATH` in the compose `.env` on the box.

### Before any of that

```bash
cp caddy/Caddyfile caddy/Caddyfile.bak.$(date +%F)
cp docker/compose.prod.yml docker/compose.prod.yml.bak.$(date +%F)
```

Then `docker compose -f docker/compose.prod.yml up -d --build robotics-course`
and reload Caddy. Check the backup still contains Delivery Manager's
path and the client-site paths before reloading — if it does not, stop.

## What is NOT served

The simulation. Gazebo needs far more than this box has — 1 vCPU and
2 GB against a simulator that wants 6 GB and four cores — and
`docs/learning-path.md` is honest that the simulation runs on your own
machine. The site is the reading half of the course. The doing half
stays local, which is also where the robot is.

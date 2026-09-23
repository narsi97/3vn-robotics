# Deploying

Two halves, deployed completely differently, and the asymmetry is
deliberate.

| | Where | How | Why |
|---|---|---|---|
| **robot runtime** | on/next to the arm | pull a published image by digest | a robot is remote; rollback must be a pull |
| **fleet view** | the shared 3VN VPS | built on the VPS from a sibling checkout | that is how that host already works |

One exception is cheaper than two deployment models. The robot genuinely
needs registry-based deployment — it is somewhere else, and rolling back
by rebuilding on it is not a thing. The VPS already builds every other
3VN product in place, and adding a second pattern there would buy
nothing.

## What does NOT run on the VPS

**The robot.** 50 Hz servo control cannot cross the internet, and the
robot must keep working when the VPS is unreachable — losing the network
degrades observability, never motion. Verified: killing the fleet service
leaves `/readyz` at 200 and scenarios passing, with one throttled
warning in the log.

**Gazebo.** The shared VPS is 1 vCPU / 2 GB already carrying about ten
containers plus Umami. Simulation needs roughly 4 vCPU / 8 GB. It stays
local; cloud simulation, if ever wanted, is free on GitHub Actions
runners.

**The robot's own dashboard.** It runs on the robot, on purpose. A
dashboard that needs the internet to tell you the arm is fine is a
dashboard that lies to you during an outage.

## Robots report out; nothing reaches in

```
robot ──POST /api/telemetry──> VPS fleet view
  │                              │
  └── keeps running regardless    └── shows what it last heard, and
                                     says so when that was a while ago
```

Robots sit behind home NAT, so inbound connections do not work anyway.
But the real reason is the second one: a robot must never wait on this.

The fleet view shows a robot as **stale** after 35 s of silence and
refuses to report it ready — a robot that stopped reporting is not
healthy just because its last message said so. That would turn an outage
into a silent one, which is worse than having no dashboard.

## Shared VPS: the fleet view

On your machine, push this repo and make sure the VPS can clone it. Then
on the VPS, as a sibling of `3vnsystems-infrastructure`:

```bash
git clone https://github.com/narsi97/3vn-robotics.git ~/3vn-robotics
```

Then, in the `3vnsystems-infrastructure` checkout:

1. Append the service in [`shared-vps/compose.fragment.yml`](shared-vps/compose.fragment.yml)
   to `docker/compose.prod.yml`, and add it to the `caddy` service's
   `depends_on`.
2. Insert the blocks from [`shared-vps/Caddyfile.fragment`](shared-vps/Caddyfile.fragment)
   into `caddy/Caddyfile`, **above** the catch-all `handle` at the end.
3. Append [`shared-vps/root-env.additions`](shared-vps/root-env.additions)
   to `.env` (and to `.env.prod.example`).
4. Copy [`shared-vps/threevn-fleet.env.example`](shared-vps/threevn-fleet.env.example)
   to `env/threevn-fleet.env`, generate a token with
   `openssl rand -base64 48`, and `chmod 600` it.
5. Add `Disallow: /fleet-` to `3vnsystems-homepage/robots.txt`.
6. Validate and reload Caddy rather than restarting it:

   ```bash
   docker compose -f docker/compose.prod.yml --env-file .env exec caddy \
     caddy validate --config /etc/caddy/Caddyfile --adapter caddyfile
   docker compose -f docker/compose.prod.yml --env-file .env exec caddy \
     caddy reload   --config /etc/caddy/Caddyfile --adapter caddyfile
   ```

7. Build and start just this service:

   ```bash
   docker compose -f docker/compose.prod.yml --env-file .env build threevn-fleet
   docker compose -f docker/compose.prod.yml --env-file .env up -d threevn-fleet
   ```

**Before touching `caddy/Caddyfile` or `compose.prod.yml`, read
`3vnsystems-infrastructure/ARCHITECTURE.md` §5.** The VPS copies of both
contain blocks that were never committed — Delivery Manager's secret path
and the client sites. Treat the VPS as authoritative, patch in place, and
back up first. Copying a local file over either will delete live config.

### It is unlisted, and that is the whole access control

The path is a secret. Do not paste it anywhere public, do not link it
from the homepage, and add it to `robots.txt`. The page also sends
`X-Robots-Tag: noindex, nofollow` so a leaked path does not become an
indexed one.

The **ingest** endpoint is separately protected by a bearer token,
because unlike the view it can change what the fleet page says. Anything
able to write to it can lie about a robot.

## Enrolling a robot

Set three variables wherever the runtime container runs:

```bash
THREEVN_FLEET_URL=https://3vnsystems.com/fleet-fab0308e05e11b540b5324b1/api/telemetry
THREEVN_FLEET_TOKEN=<the token from threevn-fleet.env>
THREEVN_ROBOT_ID=robot-001
```

With `THREEVN_FLEET_URL` unset nothing starts and nothing is sent — a
robot on a bench should not be phoning home. With a URL but no token it
refuses to start and says why, rather than retrying into a 401 every ten
seconds for the life of the robot.

## Robot runtime: see the other half

Deploying and rolling back the robot image is in
[`../docs/deployment.md`](../docs/deployment.md). The short version:
pull an immutable `sha-` tag. Rolling back never requires a rebuild.

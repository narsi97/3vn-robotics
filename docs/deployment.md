# Deployment strategy

> Phases 6–8. Nothing is deployed today; `deploy/` holds a README
> explaining exactly that.

## What runs where

| | Where | Why |
|---|---|---|
| ROS 2 control loop | **on/next to the robot** | 50 Hz servo control cannot cross the internet |
| Gazebo | dev machine or CI | 4 vCPU / 8 GB minimum |
| Dashboard, API, telemetry | VPS | no real-time requirement |

**The robot must keep working when the VPS is unreachable.** Losing the
network degrades observability and remote management, never motion.

## The shared VPS cannot run simulation

1 vCPU / 2 GB, already carrying ~10 containers plus Umami. Gazebo needs
roughly 4 vCPU / 8 GB. Simulation stays local; cloud Gazebo, if ever
wanted, is free on GitHub Actions public runners.

## The CSP blocker: resolved by design, not by exception

Flagged in Phase 1: the shared 3VN Caddy sets `connect-src 'self'` for
**every** product on the domain, so a dashboard talking to a
`rosbridge`/`foxglove_bridge` WebSocket on another origin would be
blocked.

**Phase 3 removed the problem instead of working around it.** The
dashboard uses Server-Sent Events over plain HTTP rather than a
WebSocket, and serves its own static page. Proxied under the same origin,
that satisfies `connect-src 'self'` with **no CSP change at all** — so
no `Report-Only` rollout, and no policy loosened for the other five
products.

Adding the dashboard to the shared Caddy is then the ordinary two-block
pattern every other 3VN product uses:

```caddyfile
@robotics path /robotics/api/*
handle @robotics {
    uri strip_prefix /robotics
    reverse_proxy threevn-dashboard:8107
}
handle /robotics/* {
    reverse_proxy threevn-dashboard:8107
}
```

One caveat for whoever wires this up: SSE must not be buffered. The
handler already sends `X-Accel-Buffering: no`, and Caddy's
`reverse_proxy` streams by default — but verify it, because a buffered
stream looks exactly like a frozen dashboard.

**This is still not deployed.** The dashboard runs on the robot, and the
robot is not on the VPS. What lands on the VPS in Phase 8 is telemetry
ingestion and a remote view, not this node.

## Images and rollback

The runtime image is published to GHCR on every push to `main`:

```
ghcr.io/narsi97/3vn-robotics:latest        moves
ghcr.io/narsi97/3vn-robotics:sha-<short>   immutable
ghcr.io/narsi97/3vn-robotics:v1.2.3        on a release tag
```

Built for **linux/amd64 and linux/arm64**, so the same tag runs on a mini
PC or on a Raspberry Pi beside the arm, natively either way.

### Rolling back is a pull

```bash
docker pull ghcr.io/narsi97/3vn-robotics:sha-b708737
docker stop threevn-runtime && docker rm threevn-runtime
docker run -d --name threevn-runtime --restart unless-stopped \
  --device /dev/ttyUSB0 \
  -p 127.0.0.1:8107:8107 \
  ghcr.io/narsi97/3vn-robotics:sha-b708737
```

That is the whole procedure, and it is deliberate. Spec §22 requires that
returning to a previous version never depends on rebuilding: **if the only
way back is a successful build, you do not have a rollback, you have a
hope.** The `sha-` tags are immutable, so the version you roll back to is
the version that was tested.

`docker inspect` answers what a container is running without starting
anything, because the image carries OCI labels:

```bash
docker inspect --format '{{index .Config.Labels "org.opencontainers.image.revision"}}' \
  ghcr.io/narsi97/3vn-robotics:latest
```

and the running robot answers the same question at `/api/version`.

## Versioning

Every deployment must expose:

```json
{
  "robot_id":         "robot-001",
  "software_version": "0.1.0",
  "firmware_version": "0.0.0",
  "model_version":    "threevn_arm_v1",
  "config_version":   "...",
  "git_commit":       "...",
  "deployed_at":      "..."
}
```

Without this, debugging a physical system is guesswork: you cannot tell
which code produced the behaviour you are looking at.

## Rollback

Deployments are versioned images; rollback re-points at the previous tag.
**Rollback must never require rebuilding.** If the only way back is a
successful build, you have no rollback — you have a hope.

## Approval gates

Hardware deployment is never automatic on merge. A robot that moves on
every merged PR is a hazard.

## House conventions this will follow

The other 3VN products ship a `deploy/` directory with
`Caddyfile.fragment`, `compose.fragment.yml`, `<product>.env.example` and
a DB-create script, plus a `standalone/` variant for a dedicated host.
Robotics will follow the same shape in Phase 8. Config in Git; state and
secrets on the server only.

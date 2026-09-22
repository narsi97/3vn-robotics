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

## Known blocker: CSP will block the dashboard WebSocket

The shared 3VN Caddy sets `connect-src 'self'` for **every** product on
the domain. A dashboard connecting to a `rosbridge` or `foxglove_bridge`
WebSocket on a different origin will be blocked by CSP.

Two options, decided in Phase 3:

1. **Proxy the WebSocket under the same origin** via Caddy. Preferred —
   no CSP change, so no other product is affected.
2. **Add a targeted `connect-src` entry**, shipped as
   `Content-Security-Policy-Report-Only` first and folded into the
   enforcing policy once real traffic shows zero violations — per that
   Caddyfile's own documented practice.

Flagged here so it is a design input rather than a demo-day surprise.

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

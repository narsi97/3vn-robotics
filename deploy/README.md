# Deployment

**There is nothing here yet, and that is deliberate.**

The other 3VN products each ship a `deploy/` directory with a
`Caddyfile.fragment`, a `compose.fragment.yml`, an env example and a
database-create script. This one is empty because **nothing in this
project is deployed to a server yet, and most of it never will be.**

## What will and will not run on the VPS

| | Where | Why |
|---|---|---|
| ROS 2 control loop | on/next to the robot | 50 Hz servo control cannot cross the internet |
| Gazebo | dev machine or CI | needs ~4 vCPU / 8 GB; the shared VPS has 1 vCPU / 2 GB |
| Dashboard, telemetry ingest | VPS | no real-time requirement |

The shared 3VN VPS **cannot run Gazebo**. It is a 1 vCPU / 2 GB box
already carrying around ten containers plus Umami. Simulation stays local;
cloud simulation, if ever wanted, is free on GitHub Actions public
runners.

## What lands here, and when

**Phase 3** — the dashboard, the first thing with a server side. That
brings the standard fragment trio, plus one known problem: the shared
Caddy sets `connect-src 'self'` for every product on the domain, so a
`rosbridge`/`foxglove_bridge` WebSocket on another origin will be blocked
by CSP. See [`../docs/deployment.md`](../docs/deployment.md).

**Phase 8** — robot-side deployment: versioned images, rollback that never
requires a rebuild, and an approval gate before anything reaches physical
hardware.

An explained absence is worth a file. An empty scaffold is not.

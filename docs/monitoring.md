# Observability

```
  robot ──OTLP/HTTP──> collector ──remote-write──> VictoriaMetrics ──> Grafana
          push, 15s                                   90d retention      SSH tunnel
```

Robots **push**. Nothing scrapes them: they sit behind home NAT where
inbound connections do not work, and more fundamentally a robot must
never wait on a monitoring backend.

## Why OTLP, and why not the SDK

OTLP was chosen for exactly one property: **instrument once and the
backend becomes swappable**. VictoriaMetrics today, something else later,
by editing a collector config rather than re-instrumenting a robot.

The robot emits OTLP/HTTP with a **JSON body, built with the standard
library**. That is part of the OTLP specification, and it was verified
against a real OpenTelemetry Collector before a line of the exporter was
written — not assumed.

The SDK was measured and declined: roughly **20 MB and a dozen packages**,
and the ROS image has no `pip` at all, so it would mean adding one.
Phase 6 spent real effort getting the robot image to 1.67 GB by removing
what it does not need, and this is the constrained, safety-adjacent
machine that will eventually sit on a network.

**The honest cost: no traces, and no auto-instrumentation.** Those are
what the SDK is genuinely good at. When a trace spanning perception →
planning → action is actually wanted (Phase 11+), that is when the
dependency earns itself. Metrics do not need it.

## Running it

```bash
cd monitoring && docker compose up -d
ssh -L 3000:127.0.0.1:3000 root@<monitoring-host>   # then localhost:3000
```

Grafana binds **127.0.0.1 only** and is reached over an SSH tunnel —
the same access model as Umami on the product VPS. No public login page,
nothing to brute force, and no window where default credentials are
exposed.

Only one port faces robots: the collector's OTLP endpoint. Put TLS in
front of it before pointing a robot on the open internet at it.

## Why its own box

The shared product VPS is 1 vCPU / 2 GB already carrying ~10 containers
plus Umami. This stack needs roughly 650 MB and would not fit.

The better reason: **monitoring that dies with the host it monitors tells
you nothing at the moment you need it.**

VictoriaMetrics rather than Prometheus — about a tenth the memory for the
same PromQL surface, which is what makes the whole thing fit on 4 GB. The
collector's `memory_limiter` is first in the pipeline on purpose: under
load it refuses data rather than growing until the kernel kills it, and
an OOM there would take the entire observability stack down exactly when
someone needed it.

## Enrolling a robot

```bash
THREEVN_OTLP_ENDPOINT=http://<host>:4318/v1/metrics
THREEVN_OTLP_TOKEN=<optional bearer token>
THREEVN_OTLP_INTERVAL=15
```

Unset means nothing starts and nothing is sent. A robot on a bench should
not be trying to reach a monitoring stack.

## Metric names are translated, and it bites

The collector rewrites OTel names into Prometheus conventions, appending
a suffix derived from **unit and type**:

| emitted as | arrives as |
|---|---|
| `threevn.robot.ready`, unit `1` | `threevn_robot_ready_ratio` |
| `threevn.robot.uptime`, unit `s` | `threevn_robot_uptime_seconds` |
| `threevn.joint_states.received`, counter | `threevn_joint_states_received_total` |
| `threevn.controllers.active`, unit `{controller}` | `threevn_controllers_active` |

This is not trivia. The first dashboard queried the OTel names and every
panel was empty — a working pipeline behind a dashboard that showed
nothing. And a count of three controllers declared with unit `1` arrived
as `..._ratio`, which is simply wrong. **Units are load-bearing here.**

`resource_to_telemetry_conversion: enabled: true` is equally load-bearing:
without it `robot.id`, `git.commit` and the version labels are dropped
and every robot looks identical.

## Grafana provisioning has one trap

The datasource **must** declare an explicit `uid`. Dashboards reference a
datasource by uid, and without one Grafana generates a random value per
install — so a provisioned dashboard points at a datasource that does not
exist and every panel fails to load with no useful message. Observed:
Grafana had assigned `P4169E866C3094E38`.

## What is deliberately absent

**Logs and traces.** Loki and Tempo slot into the same collector when
they are needed; nothing about this design has to change. Adding them
now would spend memory on a 4 GB box for data nobody is reading yet.

**Alerting.** Rules belong once there is a robot whose behaviour is known
well enough to say what "wrong" looks like. Alerting on a system with no
baseline mostly produces noise, and an alarm that cries wolf gets ignored.

The signal to alert on, when that time comes, is `threevn_robot_ready_ratio`
— not liveness. Liveness only says a process answered.

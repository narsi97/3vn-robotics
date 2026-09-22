# Observability strategy

> Phase 9. Documented now because the decisions constrain earlier phases.

## OpenTelemetry is the load-bearing choice

Every product — Go backends, Expo frontends, ROS 2 nodes, and the ESP32
via its host gateway — emits **OTLP**.

```
  app / ROS 2 node / robot
            │ OTLP
            ▼
  OpenTelemetry Collector        (~150 MB)
            │
   ┌────────┼────────┐
VictoriaMetrics  Loki  Tempo
   metrics      logs  traces
   └────────┼────────┘
         Grafana        ← SSH tunnel only
```

Instrument once against OTLP and the backend becomes swappable by editing
an exporter config — never by re-instrumenting six codebases. That is the
whole point: the storage decision stops being permanent.

**VictoriaMetrics rather than Prometheus** — roughly 10× lighter for the
same PromQL surface, which is what makes the whole stack fit in ~650 MB.

## Why this suits robotics specifically

- **Traces matter more here than in CRUD.** One trace spanning perception
  → planning → action → hardware is exactly the "investigate a failure
  from dashboard to logs to deployment version to git commit" exercise the
  course is built around. Metrics alone cannot show that chain.
- **The robot survives the VPS.** The Collector buffers and retries
  locally, so losing the network degrades observability, not motion. The
  robot must never depend on the internet to move.
- **One telemetry model for simulation and hardware.** Both emit through
  the same SDK, so a Grafana dashboard built against Gazebo works
  unchanged against the physical arm.

## Hosting

Self-hosted on a small dedicated box (2 vCPU / 4 GB, ~€5.50/mo). Nothing
leaves 3VN infrastructure — which matters because the estate includes a
product handling bank data.

It cannot go on the existing shared VPS: 2 GB already carrying ~10
containers plus Umami leaves roughly 300–500 MB headroom, and the stack
needs ~650 MB. A separate box is also better practice — monitoring that
dies with the host it monitors tells you nothing at the moment you need it.

**Access follows the existing Umami precedent:** Grafana binds
`127.0.0.1` and is reached over `ssh -L`. No public login page, nothing to
brute-force.

## Planned signals

| Metric | Why |
|---|---|
| `joint_command_count`, `joint_error_count` | command health |
| `command_latency`, `movement_duration` | is the arm keeping up |
| `connection_status`, heartbeat age | ESP32 link |
| `servo_temperature`, `battery_voltage` | hardware health |
| `controller_update_rate` | is the 50 Hz loop actually 50 Hz |
| `simulation_rtf` | real-time factor |
| model name / version / checksum | which model is the robot running |

Every emission carries `robot_id`, `software_version`, `firmware_version`
and `git_commit` as resource attributes. Without those, a metric from a
fleet is unattributable.

## Deliberately not

No Elasticsearch, no Kafka, no Kubernetes. Each would need to solve a
problem this system actually has.

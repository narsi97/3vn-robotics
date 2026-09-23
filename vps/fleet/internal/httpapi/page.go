// Copyright 2026 3VN Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package httpapi

// fleetHTML is the whole page. No framework, no CDN, no build step.
//
// It is server-rendered and refreshes itself, rather than fetching JSON,
// so it satisfies the shared Caddy's `connect-src 'self'` with nothing to
// configure — the same reasoning that made the robot's own dashboard use
// SSE instead of a WebSocket.
const fleetHTML = `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="robots" content="noindex, nofollow">
<meta http-equiv="refresh" content="10">
<title>3VN Fleet</title>
<style>
  :root {
    --bg:#14161a; --panel:#1c1f25; --line:#2b2f37; --text:#e6e8ec;
    --muted:#8b93a1; --ok:#3fb950; --warn:#d29922; --bad:#f85149;
    --accent:#e67e22;
    --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  }
  *{box-sizing:border-box}
  body{margin:0;padding:24px 16px;background:var(--bg);color:var(--text);
       font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
  .wrap{max-width:900px;margin:0 auto}
  h1{font-size:20px;margin:0 0 4px;font-weight:600}
  h1 span{color:var(--accent)}
  .sub{color:var(--muted);font-size:13px;margin-bottom:18px}
  .card{background:var(--panel);border:1px solid var(--line);
        border-radius:10px;padding:14px 16px;margin-bottom:12px}
  .head{display:flex;align-items:baseline;gap:10px;flex-wrap:wrap;
        margin-bottom:10px}
  .id{font:600 15px var(--mono)}
  .pill{font:600 11px/1 var(--mono);text-transform:uppercase;
        letter-spacing:.08em;padding:4px 8px;border-radius:999px;
        border:1px solid var(--line)}
  .ok{color:var(--ok)} .bad{color:var(--bad)} .warn{color:var(--warn)}
  table{width:100%;border-collapse:collapse;font:12.5px var(--mono)}
  td{padding:3px 0}
  td.k{color:var(--muted);padding-right:14px;white-space:nowrap;width:40%}
  td.v{text-align:right}
  .reasons{margin:8px 0 0;padding-left:18px;color:var(--bad);font-size:12.5px}
  .empty{color:var(--muted);text-align:center;padding:36px 0}
  footer{color:var(--muted);font-size:12px;margin-top:16px}
</style>
</head>
<body>
<div class="wrap">
  <h1>3<span>VN</span> Fleet</h1>
  <div class="sub">Robots report in. This page never reaches out to them.</div>

  {{if not .Robots}}
  <div class="card empty">
    No robot has reported yet.<br>
    <span style="font-size:12.5px">
      Set THREEVN_FLEET_URL and THREEVN_FLEET_TOKEN on a robot to enrol it.
    </span>
  </div>
  {{end}}

  {{range .Robots}}
  <div class="card">
    <div class="head">
      <span class="id">{{.RobotID}}</span>
      {{if .Stale}}
        <span class="pill bad">no contact</span>
      {{else if .Ready}}
        <span class="pill ok">ready</span>
      {{else}}
        <span class="pill bad">not ready</span>
      {{end}}
      <span class="pill">{{if .HardwareTarget}}{{.HardwareTarget}}{{else}}unknown{{end}}</span>
    </div>
    <table>
      <tr><td class="k">last report</td>
          <td class="v {{if .Stale}}bad{{end}}">{{printf "%.0f" .AgeSeconds}}s ago</td></tr>
      <tr><td class="k">software</td><td class="v">{{.SoftwareVersion}}</td></tr>
      <tr><td class="k">firmware</td>
          <td class="v {{if eq .FirmwareVersion "unknown"}}warn{{end}}">{{.FirmwareVersion}}</td></tr>
      <tr><td class="k">commit</td><td class="v">{{.GitCommit}}</td></tr>
      <tr><td class="k">model</td><td class="v">{{.RobotModel}}</td></tr>
      <tr><td class="k">uptime</td><td class="v">{{printf "%.0f" .UptimeSeconds}}s</td></tr>
      <tr><td class="k">joint messages</td><td class="v">{{.JointMessages}}</td></tr>
    </table>
    {{if and .Reasons (not .Stale)}}
    <ul class="reasons">{{range .Reasons}}<li>{{.}}</li>{{end}}</ul>
    {{end}}
    {{if .Stale}}
    <ul class="reasons">
      <li>nothing received recently; whatever it last said is no longer evidence</li>
    </ul>
    {{end}}
  </div>
  {{end}}

  <footer>
    Refreshes every 10s. Losing this page does not affect any robot &mdash;
    reports are one-way and a robot never waits on a reply.
  </footer>
</div>
</body>
</html>`

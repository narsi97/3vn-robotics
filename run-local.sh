#!/usr/bin/env bash
# Bring up the dev environment and open RViz.
#
# House convention: every 3VN product ships a run-local.sh plus a
# .claude/launch.json pointing at it.
set -euo pipefail
cd "$(dirname "$0")"

if ! docker info >/dev/null 2>&1; then
  echo "Starting Docker Desktop..."
  open -a Docker
  until docker info >/dev/null 2>&1; do sleep 2; done
fi

if ! docker image inspect threevn-robotics-dev:local >/dev/null 2>&1; then
  echo "First run: building the image. This takes a while."
  make setup
fi

make up
make build
echo ""
echo "  RViz -> http://localhost:8106/vnc.html"
echo ""
exec make view

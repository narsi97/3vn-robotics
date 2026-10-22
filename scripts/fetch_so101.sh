#!/usr/bin/env bash
# Fetch the upstream SO-101 description into vendor/, which is gitignored.
#
# The arm is TheRobotStudio's SO-ARM100/SO-101, Apache-2.0. Apache-2.0
# permits commercial use, which is the whole reason this is usable here
# and EEZYbotARM was not: that one is CC BY-NC, and a NonCommercial
# licence and a paid course do not mix.
#
# NOT vendored into git. The STLs are binary blobs and this repository
# does not carry those -- a design rule from Phase 0, and the reason the
# 3VN arm is built from primitives. Fetching keeps the rule and keeps
# the upstream copy authoritative rather than forked by accident.
set -euo pipefail

REPO=https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main
SIM=Simulation/SO101
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$HERE/vendor/so101"

mkdir -p "$OUT/assets"
echo "  fetching $SIM/so101_new_calib.urdf"
curl -fsSL "$REPO/$SIM/so101_new_calib.urdf" -o "$OUT/so101.urdf"

# Pull exactly the meshes this URDF names, rather than guessing a list
# that upstream can change underneath us.
grep -o 'filename="assets/[^"]*"' "$OUT/so101.urdf" \
  | sed 's/filename="assets\///;s/"$//' | sort -u | while read -r mesh; do
    echo "  fetching assets/$mesh"
    curl -fsSL "$REPO/$SIM/assets/$mesh" -o "$OUT/assets/$mesh"
done

cat > "$OUT/NOTICE" <<'NOTICE'
SO-ARM100 / SO-101 robot description and meshes
Copyright TheRobotStudio and contributors
Licensed under the Apache License, Version 2.0
Source: https://github.com/TheRobotStudio/SO-ARM100

Fetched unmodified by scripts/fetch_so101.sh. 3VN adds ros2_control and
Gazebo tags in a separate wrapper (src/threevn_sim/urdf/so101_gz.urdf.xacro)
and does not modify these files.
NOTICE

echo "  $(ls "$OUT/assets" | wc -l | tr -d ' ') meshes in $OUT/assets"
echo "  ok -> $OUT"

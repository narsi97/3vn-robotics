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

# PINNED to one upstream commit, and the URDF is checksummed.
#
# This used to fetch from `main`. That made every build depend on
# whatever upstream had pushed that morning: a renamed joint there would
# have failed our tests here with nothing changed in this repository,
# and a moved limit would have changed what the simulated arm does with
# nobody deciding it should. A third-party description is a dependency,
# and a dependency gets a version.
#
# To take a newer upstream: set REF to the new commit, run this, read
# the diff of vendor/so101/so101.urdf, run `make test-ros
# PROFILE=so101_follower`, and only then update URDF_SHA256.
REF="${SO101_REF:-5f6d2b876a53a4872e405b991dd925556c9e38a4}"
URDF_SHA256="${SO101_URDF_SHA256:-3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c}"

REPO="https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/$REF"
SIM=Simulation/SO101
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$HERE/vendor/so101"

sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | cut -d' ' -f1
  else
    shasum -a 256 "$1" | cut -d' ' -f1
  fi
}

mkdir -p "$OUT/assets"
echo "  fetching $SIM/so101_new_calib.urdf @ ${REF:0:12}"
curl -fsSL "$REPO/$SIM/so101_new_calib.urdf" -o "$OUT/so101.urdf.part"

GOT="$(sha256 "$OUT/so101.urdf.part")"
if [ "$GOT" != "$URDF_SHA256" ]; then
  rm -f "$OUT/so101.urdf.part"
  echo "" >&2
  echo "  so101.urdf does not match its pinned checksum." >&2
  echo "    expected $URDF_SHA256" >&2
  echo "    got      $GOT" >&2
  echo "  Nothing was installed. If REF was changed on purpose, review" >&2
  echo "  the new file and update URDF_SHA256 in $0." >&2
  exit 1
fi
mv "$OUT/so101.urdf.part" "$OUT/so101.urdf"

# Pull exactly the meshes this URDF names, rather than guessing a list
# that upstream can change underneath us.
grep -o 'filename="assets/[^"]*"' "$OUT/so101.urdf" \
  | sed 's/filename="assets\///;s/"$//' | sort -u | while read -r mesh; do
    echo "  fetching assets/$mesh"
    curl -fsSL "$REPO/$SIM/assets/$mesh" -o "$OUT/assets/$mesh"
done

cat > "$OUT/NOTICE" <<NOTICE
SO-ARM100 / SO-101 robot description and meshes
Copyright TheRobotStudio and contributors
Licensed under the Apache License, Version 2.0
Source: https://github.com/TheRobotStudio/SO-ARM100
Commit: $REF
so101.urdf sha256: $URDF_SHA256

Fetched unmodified by scripts/fetch_so101.sh. 3VN adds a world anchor,
ros2_control and Gazebo tags, servo-derived limits and named poses in a
separate overlay, built in memory at launch from
src/threevn_robot_description/config/so101_follower.yaml, and does not
modify these files.
NOTICE

echo "  $(ls "$OUT/assets" | wc -l | tr -d ' ') meshes in $OUT/assets"
echo "  ok -> $OUT"

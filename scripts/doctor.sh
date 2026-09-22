#!/usr/bin/env bash
# Preflight for the 3VN Robotics dev environment.
#
# Every check here corresponds to a failure that is confusing when it
# happens later: a Gazebo process OOM-killed by the Docker VM looks like a
# Gazebo bug, an emulated image looks like "Docker is slow", a full disk
# looks like a broken apt mirror. Fail here instead, with the fix.
set -uo pipefail

FAIL=0
ok()   { printf '  \033[32mok\033[0m    %s\n' "$1"; }
warn() { printf '  \033[33mwarn\033[0m  %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FAIL=1; }

echo ""
echo "3VN Robotics -- environment check"
echo ""

# --- Docker daemon -------------------------------------------------------
if ! command -v docker >/dev/null 2>&1; then
  bad "docker not installed. Install Docker Desktop: https://docker.com/products/docker-desktop"
elif ! docker info >/dev/null 2>&1; then
  bad "Docker daemon not reachable. Start Docker Desktop (open -a Docker) and re-run."
else
  ok "docker daemon reachable ($(docker --version | cut -d, -f1))"

  # --- Architecture ------------------------------------------------------
  HOST_ARCH="$(uname -m)"
  DOCKER_ARCH="$(docker info --format '{{.Architecture}}' 2>/dev/null)"
  case "$HOST_ARCH:$DOCKER_ARCH" in
    arm64:aarch64|aarch64:aarch64|x86_64:x86_64)
      ok "architecture $DOCKER_ARCH, native (no emulation)" ;;
    *)
      warn "host is $HOST_ARCH but docker reports $DOCKER_ARCH -- images may run under QEMU, which is very slow." ;;
  esac

  # --- VM memory ---------------------------------------------------------
  MEM_BYTES="$(docker info --format '{{.MemTotal}}' 2>/dev/null || echo 0)"
  MEM_GB=$(( MEM_BYTES / 1000000000 ))
  if [ "$MEM_BYTES" -eq 0 ]; then
    warn "could not read Docker VM memory"
  elif [ "$MEM_GB" -lt 6 ]; then
    bad "Docker VM has ${MEM_GB}GB RAM; Gazebo needs >= 6GB.
        Fix: Docker Desktop > Settings > Resources > Memory -> 6GB or more, then Apply & Restart."
  else
    ok "docker VM memory ${MEM_GB}GB"
  fi

  # --- VM cpus -----------------------------------------------------------
  NCPU="$(docker info --format '{{.NCPU}}' 2>/dev/null || echo 0)"
  if [ "$NCPU" -lt 4 ]; then
    warn "Docker VM has ${NCPU} CPUs. Gazebo renders on llvmpipe (software GL); 4+ is strongly recommended.
        Fix: Docker Desktop > Settings > Resources > CPUs."
  else
    ok "docker VM cpus ${NCPU}"
  fi
fi

# --- Disk ----------------------------------------------------------------
# The image is ~5-6GB, plus build/install volumes. 12GB is the floor.
AVAIL_GB="$(df -g / 2>/dev/null | awk 'NR==2 {print $4}')"
if [ -z "${AVAIL_GB:-}" ]; then
  AVAIL_GB="$(df -BG / 2>/dev/null | awk 'NR==2 {gsub(/G/,"",$4); print $4}')"
fi
if [ -n "${AVAIL_GB:-}" ] && [ "$AVAIL_GB" -lt 12 ] 2>/dev/null; then
  bad "only ${AVAIL_GB}GB free. The image is ~5-6GB plus build volumes; 12GB is the floor.
        Fix: free space, or 'make nuke' to drop old images and volumes."
elif [ -n "${AVAIL_GB:-}" ]; then
  ok "disk ${AVAIL_GB}GB free"
fi

# --- Port ----------------------------------------------------------------
if lsof -nP -iTCP:8106 -sTCP:LISTEN >/dev/null 2>&1; then
  if docker ps --format '{{.Names}} {{.Ports}}' 2>/dev/null | grep -q 'threevn-ros.*8106'; then
    ok "port 8106 held by our own container"
  else
    bad "port 8106 is in use by another process. noVNC cannot bind.
        Fix: lsof -nP -iTCP:8106 -sTCP:LISTEN   then stop that process."
  fi
else
  ok "port 8106 free"
fi

# --- git / gh ------------------------------------------------------------
command -v git >/dev/null 2>&1 && ok "git $(git --version | awk '{print $3}')" \
                               || bad "git not installed"
if command -v gh >/dev/null 2>&1; then
  ok "gh $(gh --version | head -1 | awk '{print $3}')"
else
  warn "gh not installed -- only needed to create the GitHub repo.
        Fix: brew install gh   (or create narsi97/3vn-robotics in the browser)"
fi

echo ""
if [ "$FAIL" -ne 0 ]; then
  echo "  Environment is not ready. Fix the FAIL lines above, then re-run 'make doctor'."
  echo ""
  exit 1
fi
echo "  Environment looks good. Next: make setup"
echo ""

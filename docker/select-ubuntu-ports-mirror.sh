#!/usr/bin/env bash
set -euo pipefail

# Select a reachable Ubuntu Ports mirror and rewrite /etc/apt/sources.list.
# Intended for Ubuntu Focal ARM64 images such as ros:noetic-ros-base-focal.
#
# Override mirrors at build time if desired:
#   MIRRORS="https://mirror-a/ubuntu-ports https://mirror-b/ubuntu-ports" ./select-ubuntu-ports-mirror.sh
#
# Override suites if needed:
#   UBUNTU_SUITES="focal focal-updates focal-backports focal-security" ./select-ubuntu-ports-mirror.sh

SOURCES_FILE="${SOURCES_FILE:-/etc/apt/sources.list}"
CHECK_TIMEOUT="${CHECK_TIMEOUT:-15}"
UBUNTU_SUITES="${UBUNTU_SUITES:-focal focal-updates focal-backports focal-security}"

# Keep this list deliberately short and easy to maintain.
# Additional Ubuntu Ports mirrors can be injected through the MIRRORS environment variable.
DEFAULT_MIRRORS="
https://ports.ubuntu.com/ubuntu-ports
https://mirrors.ustc.edu.cn/ubuntu-ports
"

MIRRORS="${MIRRORS:-$DEFAULT_MIRRORS}"

log() {
  printf '[mirror-select] %s\n' "$*"
}

fail() {
  log "ERROR: $*"
  exit 1
}

command -v python3 >/dev/null 2>&1 || fail "python3 is required for mirror probing."
[ -f "$SOURCES_FILE" ] || fail "APT sources file not found: $SOURCES_FILE"

probe_url() {
  local url="$1"

  python3 - "$url" "$CHECK_TIMEOUT" <<'PY'
import sys
import urllib.request

url = sys.argv[1]
timeout = float(sys.argv[2])

# GET with a tiny Range request is more reliable than HEAD on some mirrors.
request = urllib.request.Request(
    url,
    headers={
        "User-Agent": "OpenMower-Mirror-Probe/1.0",
        "Range": "bytes=0-0",
    },
)

try:
    with urllib.request.urlopen(request, timeout=timeout) as response:
        status = getattr(response, "status", response.getcode())
        if 200 <= status < 400 or status == 206:
            raise SystemExit(0)
except Exception:
    raise SystemExit(1)

raise SystemExit(1)
PY
}

mirror_is_usable() {
  local mirror="$1"
  local suite
  for suite in $UBUNTU_SUITES; do
    local url="${mirror%/}/dists/${suite}/InRelease"
    log "Testing ${url}"
    if ! probe_url "$url"; then
      log "Mirror failed for suite '${suite}': ${mirror}"
      return 1
    fi
  done
  return 0
}

rewrite_sources() {
  local mirror="$1"

  cp "$SOURCES_FILE" "${SOURCES_FILE}.bak"

  # Replace the canonical Ubuntu Ports endpoint in both HTTP and HTTPS form.
  sed -i \
    -e "s|http://ports.ubuntu.com/ubuntu-ports|${mirror%/}|g" \
    -e "s|https://ports.ubuntu.com/ubuntu-ports|${mirror%/}|g" \
    "$SOURCES_FILE"

  log "Selected mirror: ${mirror%/}"
  log "Backup written to: ${SOURCES_FILE}.bak"
}

selected_mirror=""

for mirror in $MIRRORS; do
  [ -n "$mirror" ] || continue
  if mirror_is_usable "$mirror"; then
    selected_mirror="$mirror"
    break
  fi
done

[ -n "$selected_mirror" ] || fail "No configured Ubuntu Ports mirror was reachable for all required suites."

rewrite_sources "$selected_mirror"

log "Mirror selection completed."

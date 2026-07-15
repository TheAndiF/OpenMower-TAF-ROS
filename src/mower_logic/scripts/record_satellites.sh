#!/usr/bin/env bash
set -u

# Satellite/GPS logger for OpenMower.
# The live logs are written to RAM first and are copied to persistent storage
# only when this script is stopped by OpenMower, e.g. at docking/charging.
# The script intentionally does not decide start/end conditions itself.

SESSION_ID="${SAT_LOG_SESSION_ID:-$(date -u +%Y%m%d_%H%M%S)}"
RAM_BASE="${SAT_LOG_RAM_DIR:-/dev/shm/openmower_satellite_logs}"
OUTPUT_DIR="${SAT_LOG_OUTPUT_DIR:-/home/openmower/recordings/logs}"
CONTAINER_NAME="${SAT_LOG_CONTAINER:-}"

usage() {
  cat <<USAGE
Usage: $0 [--session-id ID] [--ram-path PATH] [--output-path PATH] [--container-name NAME]

Options override the SAT_LOG_* environment variables. The public configuration
is exposed below gps_state/settings and is mapped to the internal mower_logic
runtime fields before this script is started. The script therefore has no
hard-coded active configuration of its own.
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --session-id)
      SESSION_ID="${2:-}"
      shift 2
      ;;
    --ram-path)
      RAM_BASE="${2:-}"
      shift 2
      ;;
    --output-path)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    --container-name)
      CONTAINER_NAME="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done


running_inside_container() {
  [ -f /.dockerenv ] && return 0
  grep -qaE '/docker/|/kubepods/|/containerd/' /proc/1/cgroup 2>/dev/null
}

pick_first_matching_container() {
  local pattern="$1"
  docker ps --format '{{.Names}}' 2>/dev/null | grep -E "$pattern" | head -n 1
}

auto_detect_container() {
  if [ -n "${CONTAINER_NAME}" ]; then
    return
  fi
  if running_inside_container; then
    return
  fi
  if ! command -v docker >/dev/null 2>&1; then
    return
  fi

  local candidate=""
  candidate="$(pick_first_matching_container '^openmower-open_mower_ros-1$')"
  if [ -z "${candidate}" ]; then
    candidate="$(pick_first_matching_container '^OpenMowerROS$')"
  fi
  if [ -z "${candidate}" ]; then
    candidate="$(pick_first_matching_container '(^|[-_])open_?mower_?ros($|[-_])')"
  fi
  if [ -z "${candidate}" ]; then
    candidate="$(pick_first_matching_container 'openmower.*ros|ros.*openmower')"
  fi

  if [ -n "${candidate}" ]; then
    CONTAINER_NAME="${candidate}"
  fi
}

if [ -z "${SESSION_ID}" ] || [ -z "${RAM_BASE}" ] || [ -z "${OUTPUT_DIR}" ]; then
  echo "session-id, ram-path and output-path must not be empty" >&2
  exit 2
fi

auto_detect_container

SESSION_RAM_DIR="${RAM_BASE}/${SESSION_ID}"
mkdir -p "${SESSION_RAM_DIR}"

SAT_FILE="gps_satellite_list_${SESSION_ID}.log"
POS_FILE="gps_position_${SESSION_ID}.log"
ACC_FILE="gps_accuracy_${SESSION_ID}.log"
RTCM_FILE="gps_rtcm_hz_${SESSION_ID}.log"
META_FILE="satellite_logging_${SESSION_ID}.meta.json"

PIDS=()
STOPPED=0
STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

log_info() {
  echo "$*"
}

copy_to_persistent() {
  mkdir -p "${OUTPUT_DIR}"
  for file in "${SAT_FILE}" "${POS_FILE}" "${ACC_FILE}" "${RTCM_FILE}"; do
    if [ -f "${SESSION_RAM_DIR}/${file}" ]; then
      cp -f "${SESSION_RAM_DIR}/${file}" "${OUTPUT_DIR}/${file}"
    fi
  done
  cat > "${OUTPUT_DIR}/${META_FILE}" <<META
{
  "session_id": "${SESSION_ID}",
  "started_at": "${STARTED_AT}",
  "finished_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "ram_path": "${SESSION_RAM_DIR}",
  "output_path": "${OUTPUT_DIR}",
  "files": [
    "${SAT_FILE}",
    "${POS_FILE}",
    "${ACC_FILE}",
    "${RTCM_FILE}",
    "${META_FILE}"
  ]
}
META
}

stop_logger() {
  if [ "${STOPPED}" -eq 1 ]; then
    return
  fi
  STOPPED=1
  log_info "Stoppe Satelliten-/GPS-Mitschnitt und schreibe Logs in den festen Speicher..."
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  copy_to_persistent
  log_info "Logs gespeichert unter: ${OUTPUT_DIR}"
}

trap stop_logger EXIT INT TERM

run_rostopic_echo() {
  local topic="$1"
  local file="$2"
  if [ -n "${CONTAINER_NAME}" ]; then
    docker exec "${CONTAINER_NAME}" bash -lc "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; rostopic echo -p '${topic}'" > "${SESSION_RAM_DIR}/${file}" 2>&1 &
  else
    bash -lc "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; rostopic echo -p '${topic}'" > "${SESSION_RAM_DIR}/${file}" 2>&1 &
  fi
  PIDS+=("$!")
}

run_rtcm_hz() {
  if [ -n "${CONTAINER_NAME}" ]; then
    docker exec "${CONTAINER_NAME}" bash -lc "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; rostopic hz /ll/position/gps/rtcm" > "${SESSION_RAM_DIR}/${RTCM_FILE}" 2>&1 &
  else
    bash -lc "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; rostopic hz /ll/position/gps/rtcm" > "${SESSION_RAM_DIR}/${RTCM_FILE}" 2>&1 &
  fi
  PIDS+=("$!")
}

log_info "Starte GPS-/RTK-Mitschnitt in den Arbeitsspeicher"
log_info "Session: ${SESSION_ID}"
log_info "RAM-Verzeichnis: ${SESSION_RAM_DIR}"
log_info "Persistentes Ziel beim Stoppen: ${OUTPUT_DIR}"
if [ -n "${CONTAINER_NAME}" ]; then
  log_info "ROS-Container: ${CONTAINER_NAME}"
else
  log_info "ROS-Container: keiner/direkt"
fi
log_info "Dateien:"
log_info "  ${SAT_FILE}"
log_info "  ${POS_FILE}"
log_info "  ${ACC_FILE}"
log_info "  ${RTCM_FILE}"

run_rostopic_echo "/ll/position/gps/satellites" "${SAT_FILE}"
run_rostopic_echo "/ll/position/gps" "${POS_FILE}"
run_rostopic_echo "/xbot_positioning/xb_pose" "${ACC_FILE}"
run_rtcm_hz

while true; do
  sleep 1
done

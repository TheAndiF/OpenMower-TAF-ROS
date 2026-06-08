#!/usr/bin/env bash
set -u

# Satellite/GPS logger for OpenMower.
# The live logs are written to RAM first and are copied to persistent storage
# only when this script is stopped, e.g. by the MQTT cycle controller at docking.

SESSION_ID="${SAT_LOG_SESSION_ID:-$(date -u +%Y%m%d_%H%M%S)}"
RAM_BASE="${SAT_LOG_RAM_DIR:-/dev/shm/openmower_satellite_logs}"
OUTPUT_DIR="${SAT_LOG_OUTPUT_DIR:-/home/openmower/recordings/logs}"
CONTAINER_NAME="${SAT_LOG_CONTAINER:-}"
SESSION_RAM_DIR="${RAM_BASE}/${SESSION_ID}"

mkdir -p "${SESSION_RAM_DIR}"

SAT_FILE="gps_satellite_list_${SESSION_ID}.log"
POS_FILE="gps_position_${SESSION_ID}.log"
ACC_FILE="gps_accuracy_${SESSION_ID}.log"
RTCM_FILE="gps_rtcm_hz_${SESSION_ID}.log"
META_FILE="satellite_logger_${SESSION_ID}.meta.json"

PIDS=()
STOPPED=0

log_info() {
  echo "$*"
}

copy_to_persistent() {
  mkdir -p "${OUTPUT_DIR}"
  cp -f "${SESSION_RAM_DIR}/${SAT_FILE}" "${OUTPUT_DIR}/${SAT_FILE}" 2>/dev/null || true
  cp -f "${SESSION_RAM_DIR}/${POS_FILE}" "${OUTPUT_DIR}/${POS_FILE}" 2>/dev/null || true
  cp -f "${SESSION_RAM_DIR}/${ACC_FILE}" "${OUTPUT_DIR}/${ACC_FILE}" 2>/dev/null || true
  cp -f "${SESSION_RAM_DIR}/${RTCM_FILE}" "${OUTPUT_DIR}/${RTCM_FILE}" 2>/dev/null || true
  cat > "${OUTPUT_DIR}/${META_FILE}" <<META
{
  "session_id": "${SESSION_ID}",
  "ram_path": "${SESSION_RAM_DIR}",
  "path": "${OUTPUT_DIR}",
  "files": [
    "${SAT_FILE}",
    "${POS_FILE}",
    "${ACC_FILE}",
    "${RTCM_FILE}"
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
log_info "RAM-Verzeichnis: ${SESSION_RAM_DIR}"
log_info "Persistentes Ziel beim Stoppen: ${OUTPUT_DIR}"
log_info "Dateien:"
log_info "  ${SAT_FILE}"
log_info "  ${POS_FILE}"
log_info "  ${ACC_FILE}"
log_info "  ${RTCM_FILE}"

# These topics match the existing manual satellite logging workflow.
run_rostopic_echo "/ll/position/gps/satellites" "${SAT_FILE}"
run_rostopic_echo "/ll/position/gps" "${POS_FILE}"
run_rostopic_echo "/xbot_positioning/xb_pose" "${ACC_FILE}"
run_rtcm_hz

while true; do
  sleep 1
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      log_info "Warnung: ein Logger-Prozess ist nicht mehr aktiv. Mitschnitt bleibt bis zum Stoppen offen."
    fi
  done
done

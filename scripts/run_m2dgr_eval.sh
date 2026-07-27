#!/usr/bin/env bash

set -euo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_ROOT="$(cd "${TOOLS_DIR}/.." && pwd)"
ROOT_DIR="${SUPER_LIO_ROOT:-${DEFAULT_ROOT}}"
DATASET_ROOT="${DATASET_ROOT:-${ROOT_DIR}/Datasets}"
SEQUENCE="${1:-door_02}"
RATE="${2:-5.0}"
OUTPUT_DIR="${3:-${ROOT_DIR}/log/paper_eval/${SEQUENCE}}"
BAG_DIR="${DATASET_ROOT}/${SEQUENCE}_ros2"

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
  echo "Dataset not found: ${BAG_DIR}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
set +u
source /opt/ros/lyrical/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

lio_pid=""
recorder_pid=""

stop_process_group() {
  local pid="$1"
  if ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi

  kill -INT -- "-${pid}" 2>/dev/null || true
  for _ in $(seq 1 25); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.2
  done

  kill -TERM -- "-${pid}" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  if [[ -n "${recorder_pid}" ]]; then
    kill -INT "${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
  fi
  if [[ -n "${lio_pid}" ]]; then
    stop_process_group "${lio_pid}"
  fi
}
trap cleanup EXIT

setsid "${ROOT_DIR}/install/super_lio/lib/super_lio/super_lio_node" \
  --ros-args \
  --params-file "${ROOT_DIR}/install/super_lio/share/super_lio/config/M2DGR.yaml" \
  >"${OUTPUT_DIR}/super_lio.log" 2>&1 &
lio_pid="$!"

for _ in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -qx "/lio/odom"; then
    break
  fi
  sleep 0.2
done

if ! ros2 topic list 2>/dev/null | grep -qx "/lio/odom"; then
  echo "Super-LIO did not expose /lio/odom" >&2
  exit 1
fi

python3 "${TOOLS_DIR}/record_odom_tum.py" \
  "${OUTPUT_DIR}/estimated.tum" \
  >"${OUTPUT_DIR}/recorder.log" 2>&1 &
recorder_pid="$!"
sleep 0.5

start_ns="$(date +%s%N)"
ros2 bag play "${BAG_DIR}" \
  --rate "${RATE}" \
  --topics /velodyne_points /handsfree/imu \
  --disable-keyboard-controls \
  --progress-bar-update-rate 0 \
  >"${OUTPUT_DIR}/bag_play.log" 2>&1
end_ns="$(date +%s%N)"

sleep 2
cleanup
trap - EXIT

python3 -c \
  "print(f'{(${end_ns} - ${start_ns}) / 1e9:.6f}')" \
  >"${OUTPUT_DIR}/wall_time_seconds.txt"

echo "sequence=${SEQUENCE}"
echo "rate=${RATE}"
echo "poses=$(wc -l < "${OUTPUT_DIR}/estimated.tum")"
echo "wall_seconds=$(cat "${OUTPUT_DIR}/wall_time_seconds.txt")"

#!/usr/bin/env bash

set -euo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${TOOLS_DIR}/.." && pwd)"
SEQUENCE="${1:-keble-college-02}"
RATE="${2:-1.0}"
OUTPUT_DIR="${3:-${ROOT_DIR}/log/refactoring_baselines/oxford_spires_${SEQUENCE}_${RATE}x}"
DATASET_DIR="${OXFORD_SPIRES_DATASET_ROOT:-${ROOT_DIR}/Datasets/oxford_spires}"
SEQUENCE_DIR="${DATASET_DIR}/${SEQUENCE}"
DEFAULT_BAG_DIR="${SEQUENCE_DIR}/${SEQUENCE}_mcap"
if [[ ! -d "${DEFAULT_BAG_DIR}" ]]; then
  DEFAULT_BAG_DIR="${SEQUENCE_DIR}/${SEQUENCE}_ros2"
fi

export BAG_DIR="${OXFORD_SPIRES_BAG_DIR:-${DEFAULT_BAG_DIR}}"
export GROUND_TRUTH="${OXFORD_SPIRES_GROUND_TRUTH:-${SEQUENCE_DIR}/sequences/2024-03-12-keble-college-02/processed/trajectory/gt-tum.txt}"
export PARAMS_FILE="${PARAMS_FILE:-${ROOT_DIR}/src/hybrid_slam/config/point_lio_oxford_spires.yaml}"
export BAG_TOPICS="${BAG_TOPICS:-/hesai/pandar /alphasense_driver_ros/imu}"

if [[ ! -d "${BAG_DIR}" ]]; then
  echo "Oxford Spires ROS 2 bag does not exist: ${BAG_DIR}" >&2
  exit 1
fi
if [[ ! -f "${GROUND_TRUTH}" ]]; then
  echo "Oxford Spires ground truth does not exist: ${GROUND_TRUTH}" >&2
  exit 1
fi

exec "${TOOLS_DIR}/run_m2dgr_eval.sh" \
  "oxford_spires_${SEQUENCE}" "${RATE}" "${OUTPUT_DIR}"

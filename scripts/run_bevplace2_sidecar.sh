#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BEVPLACE2_ROOT="${BEVPLACE2_ROOT:-/run/media/sw/仓库/BEVPlace2}"
BEVPLACE2_PYTHON="${BEVPLACE2_PYTHON:-/home/sw/Software/anaconda3/envs/mibot/bin/python}"
BEVPLACE2_SOCKET="${BEVPLACE2_SOCKET:-/tmp/adagraph_bevplace2.sock}"
BEVPLACE2_CHECKPOINT="${BEVPLACE2_CHECKPOINT:-${BEVPLACE2_ROOT}/runs/Aug08_10-17-29/model_best.pth.tar}"

if [[ ! -x "${BEVPLACE2_PYTHON}" ]]; then
  echo "BEVPlace2 Python is not executable: ${BEVPLACE2_PYTHON}" >&2
  exit 2
fi
if [[ ! -f "${BEVPLACE2_CHECKPOINT}" ]]; then
  echo "BEVPlace2 checkpoint is missing: ${BEVPLACE2_CHECKPOINT}" >&2
  exit 2
fi

exec env CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}" \
  "${BEVPLACE2_PYTHON}" "${ROOT_DIR}/scripts/bevplace2_descriptor_service.py" \
  --socket "${BEVPLACE2_SOCKET}" \
  --source "${BEVPLACE2_ROOT}" \
  --checkpoint "${BEVPLACE2_CHECKPOINT}"

#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_JOBS="${BUILD_JOBS:-2}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Source the Jetson ROS 2 environment first:" >&2
  echo "  source /opt/ros/<distro>/setup.bash" >&2
  exit 2
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon was not found in PATH." >&2
  exit 2
fi

if ! [[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer." >&2
  exit 2
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "Warning: this host is $(uname -m), not aarch64; continuing for validation." >&2
fi

cd "${WORKSPACE_ROOT}"

if [[ "${INSTALL_DEPS:-0}" == "1" ]]; then
  if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdep was not found in PATH." >&2
    exit 2
  fi
  rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
fi

export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"

colcon build \
  --packages-up-to hybrid_slam \
  --executor sequential \
  --symlink-install \
  "$@" \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release

echo
echo "Build complete. Activate the workspace with:"
echo "  source ${WORKSPACE_ROOT}/install/setup.bash"

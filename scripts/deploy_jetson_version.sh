#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="${1:-}"

if [[ -z "${VERSION}" ]]; then
  echo "Usage: $0 <release-tag>" >&2
  echo "Example: $0 jetson-v0.2.0" >&2
  exit 2
fi

cd "${WORKSPACE_ROOT}"

if ! git remote get-url origin >/dev/null 2>&1; then
  echo "Git remote 'origin' is not configured." >&2
  exit 2
fi

if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
  echo "The Jetson worktree is not clean; deployment was stopped." >&2
  echo "Commit, back up, or remove local changes before switching versions." >&2
  exit 2
fi

git fetch --tags origin

if ! git rev-parse --verify --quiet "${VERSION}^{commit}" >/dev/null; then
  echo "Release '${VERSION}' does not exist." >&2
  exit 2
fi

git switch --detach "${VERSION}"

echo "Deploying ${VERSION} ($(git rev-parse --short HEAD))"
"${WORKSPACE_ROOT}/scripts/build_jetson.sh" --cmake-clean-cache

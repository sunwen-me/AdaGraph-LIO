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

# Record the currently deployed ref so a failed build can roll back.
# Stored under log/ (gitignored) so it does not trip the clean-tree check.
PREVIOUS_REF="$(git symbolic-ref --quiet --short HEAD || git rev-parse HEAD)"
mkdir -p "${WORKSPACE_ROOT}/log"
echo "${PREVIOUS_REF}" > "${WORKSPACE_ROOT}/log/jetson_previous_ref"

switch_to_ref() {
  local ref="$1"
  if git show-ref --verify --quiet "refs/heads/${ref}"; then
    git switch "${ref}"
  else
    git switch --detach "${ref}"
  fi
}

git switch --detach "${VERSION}"

echo "Deploying ${VERSION} ($(git rev-parse --short HEAD))"
echo "Previous ref recorded: ${PREVIOUS_REF} (log/jetson_previous_ref)"

if ! "${WORKSPACE_ROOT}/scripts/build_jetson.sh" --cmake-clean-cache; then
  echo "Build of ${VERSION} failed; rolling back to ${PREVIOUS_REF}." >&2
  switch_to_ref "${PREVIOUS_REF}"
  echo "Rebuilding ${PREVIOUS_REF} to restore a consistent install tree." >&2
  if ! "${WORKSPACE_ROOT}/scripts/build_jetson.sh" --cmake-clean-cache; then
    echo "Rebuild of ${PREVIOUS_REF} also failed." >&2
    echo "Worktree is back on ${PREVIOUS_REF}; fix the build manually." >&2
    exit 1
  fi
  echo "Rolled back to ${PREVIOUS_REF}; deployment of ${VERSION} was aborted." >&2
  exit 1
fi

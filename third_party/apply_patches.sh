#!/usr/bin/env bash
# Apply the vehicle-specific modifications to the pinned third-party packages.
# Usage: third_party/apply_patches.sh <catkin_ws>/src
set -euo pipefail
SRC_DIR="${1:?usage: apply_patches.sh <catkin_ws>/src}"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/patches" && pwd)"
for pkg in ublox gps_umd HesaiLidar_General_ROS morai_msgs; do
  echo "[patch] ${pkg}"
  git -C "${SRC_DIR}/${pkg}" apply --whitespace=nowarn "${PATCH_DIR}/${pkg}.patch"
done

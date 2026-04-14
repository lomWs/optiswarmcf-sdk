#!/usr/bin/env bash
# ============================================================
# build_backend.sh
#
# Purpose:
#   Build the ROS2 backend workspace using colcon.
#
# What it does:
#   - Sources the ROS2 environment
#   - Installs missing dependencies via rosdep
#   - Builds the workspace with colcon (--symlink-install)
#
# When to use:
#   - After cloning the repository
#   - After modifying backend ROS2 packages
#   - When dependencies change
#
# Notes:
#   - This script does NOT launch any ROS nodes
#   - It assumes a valid ROS2 installation is available
#   - It does NOT clean the workspace automatically
#
# Usage:
#   ./scripts/build_backend.sh [optional_workspace_path]
# ============================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"
WS="${1:-$REPO_ROOT/backend_ros2}"
WS="$(realpath "$WS")"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

[[ -d "$WS" ]] || die "Missing workspace: $WS"
[[ -d "$WS/src" ]] || die "Missing workspace src directory: $WS/src"

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS"

echo "=== Backend build ==="
echo "Workspace: $WS"
echo "ROS_DISTRO: $ROS_DISTRO"
echo

echo "Installing ROS dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python

echo
echo "Building workspace..."
colcon build --symlink-install

echo
echo "Backend build completed."
echo "Remember to source:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  source $WS/install/setup.bash"
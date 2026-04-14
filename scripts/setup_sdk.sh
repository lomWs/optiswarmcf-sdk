#!/usr/bin/env bash
# ============================================================
# setup_sdk.sh
#
# Purpose:
#   Initialize the Python SDK environment for the project.
#
# What it does:
#   - Creates a Python virtual environment in sdk/.venv (if missing)
#   - Activates the virtual environment
#   - Upgrades pip
#   - Installs the SDK in editable mode (pip install -e .)
#
# When to use:
#   - First time setup after cloning the repository
#   - After deleting or recreating the virtual environment
#
# Notes:
#   - This script only prepares the SDK environment
#   - It does NOT build the ROS2 backend
#   - ROS2 must still be sourced manually when running controllers
#
# Usage:
#   ./scripts/setup_sdk.sh
# ============================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"
SDK_DIR="$REPO_ROOT/sdk"
VENV_DIR="$SDK_DIR/.venv"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

[[ -d "$SDK_DIR" ]] || die "Missing SDK directory: $SDK_DIR"

cd "$SDK_DIR"

if [[ ! -d "$VENV_DIR" ]]; then
  echo "Creating virtual environment in $VENV_DIR"
  python3 -m venv .venv
else
  echo "Virtual environment already exists: $VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

python -m pip install --upgrade pip
python -m pip install -e .

echo
echo "SDK setup completed."
echo "Venv: $VENV_DIR"
echo
echo "For daily use:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  source $REPO_ROOT/backend_ros2/install/setup.bash"
echo "  source $VENV_DIR/bin/activate"
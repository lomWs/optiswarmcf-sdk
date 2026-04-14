#!/usr/bin/env bash
# ============================================================
# show_logs.sh
#
# Purpose:
#   Display backend logs in real time.
#
# What it does:
#   - Reads all log files from .runtime/logs/
#   - Streams logs using tail -f
#
# Log sources:
#   - vrpn.log        (OptiTrack / VRPN client)
#   - mocap.log       (mocap_bridge_ros2)
#   - cf_bridge.log   (Crazyflie bridge)
#
# When to use:
#   - Debugging backend issues
#   - Monitoring system behavior during experiments
#
# Notes:
#   - Requires backend to be running
#   - Stops with Ctrl+C
#
# Usage:
#   ./scripts/show_logs.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"
LOG_DIR="$REPO_ROOT/.runtime/logs"

if [[ ! -d "$LOG_DIR" ]]; then
  echo "Log directory not found: $LOG_DIR" >&2
  exit 1
fi

shopt -s nullglob
logs=("$LOG_DIR"/*.log)

if [[ ${#logs[@]} -eq 0 ]]; then
  echo "No log files found in $LOG_DIR" >&2
  exit 1
fi

echo "Showing logs from: $LOG_DIR"
tail -f "${logs[@]}"
#!/usr/bin/env bash
# ============================================================
# stop_backend.sh
#
# Purpose:
#   Stop all backend processes started by run_experiment.sh.
#
# What it does:
#   - Reads stored process IDs (VRPN, mocap, cf_bridge)
#   - Gracefully stops processes
#   - Forces termination if needed
#   - Cleans PID tracking file
#
# When to use:
#   - To manually stop the backend
#   - Before restarting with a different configuration
#   - When backend processes remain running after a crash
#
# Notes:
#   - Only affects processes started by run_experiment.sh
#   - Does NOT remove log files
#
# Usage:
#   ./scripts/stop_backend.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"
RUNTIME_DIR="$REPO_ROOT/.runtime"
PID_FILE="$RUNTIME_DIR/backend_pids.env"

if [[ ! -f "$PID_FILE" ]]; then
  echo "No backend PID file found."
  exit 0
fi

# shellcheck disable=SC1090
source "$PID_FILE"

stop_pid() {
  local pid="${1:-}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
  fi
}

force_stop_pid() {
  local pid="${1:-}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -9 "$pid" 2>/dev/null || true
  fi
}

echo "Stopping backend processes..."

# correct order is -> first cf, then mocap, then vrpn
stop_pid "${CF_PID:-}"
stop_pid "${MOCAP_PID:-}"
stop_pid "${VRPN_PID:-}"

sleep 2

force_stop_pid "${CF_PID:-}"
force_stop_pid "${MOCAP_PID:-}"
force_stop_pid "${VRPN_PID:-}"

rm -f "$PID_FILE"

echo "Backend stopped."
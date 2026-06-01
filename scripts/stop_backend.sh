#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"
RUNTIME_DIR="$REPO_ROOT/.runtime"
PID_FILE="$RUNTIME_DIR/backend_pids.env"

echo "Stopping backend processes..."

stop_pattern() {
  local pattern="$1"
  pkill -TERM -f "$pattern" 2>/dev/null || true
}

force_pattern() {
  local pattern="$1"
  pkill -KILL -f "$pattern" 2>/dev/null || true
}

# 1. prova prima dai PID salvati
if [[ -f "$PID_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$PID_FILE"

  for pid in "${CF_PID:-}" "${MOCAP_PID:-}" "${VRPN_PID:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
fi

sleep 2

# 2. fallback: stop all process matching known patterns
stop_pattern "cf_bridge_node"
stop_pattern "opti_bridge_node"
stop_pattern "vrpn_mocap"
stop_pattern "ros2 launch cf_bridge"
stop_pattern "ros2 launch opti_bridge"
stop_pattern "ros2 run vrpn_mocap"
stop_pattern "run_experiment.sh"

sleep 2

force_pattern "cf_bridge_node"
force_pattern "opti_bridge_node"
force_pattern "vrpn_mocap"
force_pattern "ros2 launch cf_bridge"
force_pattern "ros2 launch opti_bridge"
force_pattern "ros2 run vrpn_mocap"
force_pattern "run_experiment.sh"

rm -f "$PID_FILE" "$RUNTIME_DIR/backend_state.env"

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

rm -f /dev/shm/fastrtps* 2>/dev/null || true

echo "Backend stopped."
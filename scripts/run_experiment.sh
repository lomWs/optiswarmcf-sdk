#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# scripts/run_experiment.sh
#
# Robust launcher for:
# - backend_ros2 (VRPN + mocap_bridge_ros2 + cf_bridge)
# - SDK algorithm
#
# Usage:
#   ./scripts/run_experiment.sh <algorithm.py> [mocap.yaml] [cf_bridge.yaml] [vrpn_server] [vrpn_port] [workspace]
#
# Typical usage:
#   ./scripts/run_experiment.sh examples/minimal_controller.py
#
# Optional override usage:
#   ./scripts/run_experiment.sh \
#       examples/minimal_controller.py \
#       backend_ros2/src/mocap_bridge_ros2/config/mocap_alt.yaml \
#       backend_ros2/src/cf_bridge/config/cf_bridge_alt.yaml \
#       192.168.1.120 \
#       3883 \
#       backend_ros2
# ============================================================

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <algorithm.py> [mocap.yaml] [cf_bridge.yaml] [vrpn_server] [vrpn_port] [workspace]" >&2
  exit 1
fi

# ------------------------------------------------------------
# Resolve repo root from script location, not from current dir
# ------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"

# ------------------------------------------------------------
# Defaults
# ------------------------------------------------------------
DEFAULT_WS="$REPO_ROOT/backend_ros2"
DEFAULT_MOCAP_YAML="$DEFAULT_WS/src/mocap_bridge_ros2/config/mocap.yaml"
DEFAULT_CF_YAML="$DEFAULT_WS/src/cf_bridge/config/cf_bridge.yaml"
DEFAULT_VRPN_SERVER="192.168.1.100"
DEFAULT_VRPN_PORT="3883"

# ------------------------------------------------------------
# Inputs
# ------------------------------------------------------------
ALGO="$(realpath "$1")"
MOCAP_YAML="$(realpath "${2:-$DEFAULT_MOCAP_YAML}")"
CF_YAML="$(realpath "${3:-$DEFAULT_CF_YAML}")"
VRPN_SERVER="${4:-$DEFAULT_VRPN_SERVER}"
VRPN_PORT="${5:-$DEFAULT_VRPN_PORT}"
WS="$(realpath "${6:-$DEFAULT_WS}")"

SDK_DIR="$REPO_ROOT/sdk"
VENV_DIR="$SDK_DIR/.venv"

RUNTIME_DIR="$REPO_ROOT/.runtime"
LOG_DIR="$RUNTIME_DIR/logs"
STATE_FILE="$RUNTIME_DIR/backend_state.env"
PID_FILE="$RUNTIME_DIR/backend_pids.env"

mkdir -p "$LOG_DIR"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || die "Missing file: $1"
}

require_dir() {
  [[ -d "$1" ]] || die "Missing directory: $1"
}

source_ros_base() {
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"
}

source_ros_env() {
  source_ros_base
  # shellcheck disable=SC1090
  source "$WS/install/setup.bash"
}

check_inputs() {
  require_file "$ALGO"
  require_file "$MOCAP_YAML"
  require_file "$CF_YAML"
  require_dir "$WS"
  require_dir "$SDK_DIR"
  require_dir "$VENV_DIR"
  require_file "$VENV_DIR/bin/activate"
}

check_workspace_ready() {
  [[ -f "$WS/install/setup.bash" ]] || die "Workspace not built: missing $WS/install/setup.bash

Run:
  cd $WS
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --symlink-install"

  source_ros_env

  ros2 pkg list 2>/dev/null | grep -Fx "cf_bridge" >/dev/null || die "Package 'cf_bridge' not visible in ROS environment.

Run:
  cd $WS
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --symlink-install
  source install/setup.bash"

  ros2 pkg list 2>/dev/null | grep -Fx "mocap_bridge_ros2" >/dev/null || die "Package 'mocap_bridge_ros2' not visible in ROS environment.

Run:
  cd $WS
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --symlink-install
  source install/setup.bash"
}

canonical_signature() {
  printf '%s\n%s\n%s\n%s\n%s\n' \
    "$WS" "$MOCAP_YAML" "$CF_YAML" "$VRPN_SERVER" "$VRPN_PORT" \
    | sha256sum | awk '{print $1}'
}

extract_first_cf_ns() {
  python3 - "$CF_YAML" <<'PY'
import sys
try:
    import yaml
except Exception:
    yaml = None

path = sys.argv[1]

if yaml is not None:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    drones = data.get("drones", [])
    if isinstance(drones, list) and drones:
        ns = drones[0].get("ns")
        if ns:
            print(str(ns).strip())
            raise SystemExit(0)

print("/cf1")
PY
}

extract_first_mocap_topic() {
  python3 - "$CF_YAML" <<'PY'
import sys
try:
    import yaml
except Exception:
    yaml = None

path = sys.argv[1]

if yaml is not None:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    drones = data.get("drones", [])
    if isinstance(drones, list) and drones:
        topic = drones[0].get("mocap_topic")
        if topic:
            print(str(topic).strip())
            raise SystemExit(0)

print("/mocap/cf1/pose")
PY
}

FIRST_NS="$(extract_first_cf_ns)"
FIRST_NS="${FIRST_NS%/}"
FIRST_MOCAP_TOPIC="$(extract_first_mocap_topic)"
TAKEOFF_SERVICE="$FIRST_NS/takeoff"

backend_state_matches() {
  [[ -f "$STATE_FILE" ]] || return 1
  # shellcheck disable=SC1090
  source "$STATE_FILE"
  [[ "${SIGNATURE:-}" == "$(canonical_signature)" ]]
}

backend_processes_alive() {
  [[ -f "$PID_FILE" ]] || return 1
  # shellcheck disable=SC1090
  source "$PID_FILE"
  [[ -n "${VRPN_PID:-}" ]] && kill -0 "$VRPN_PID" 2>/dev/null || return 1
  [[ -n "${MOCAP_PID:-}" ]] && kill -0 "$MOCAP_PID" 2>/dev/null || return 1
  [[ -n "${CF_PID:-}" ]] && kill -0 "$CF_PID" 2>/dev/null || return 1
}

backend_ros_ready() {
  source_ros_env
  timeout 4 ros2 topic list 2>/dev/null | grep -Fx "$FIRST_MOCAP_TOPIC" >/dev/null || return 1
  timeout 4 ros2 service list 2>/dev/null | grep -Fx "$TAKEOFF_SERVICE" >/dev/null || return 1
}

write_state_files() {
  cat > "$STATE_FILE" <<EOF
SIGNATURE="$(canonical_signature)"
WS="$WS"
MOCAP_YAML="$MOCAP_YAML"
CF_YAML="$CF_YAML"
VRPN_SERVER="$VRPN_SERVER"
VRPN_PORT="$VRPN_PORT"
FIRST_NS="$FIRST_NS"
FIRST_MOCAP_TOPIC="$FIRST_MOCAP_TOPIC"
TAKEOFF_SERVICE="$TAKEOFF_SERVICE"
EOF

  cat > "$PID_FILE" <<EOF
VRPN_PID="$1"
MOCAP_PID="$2"
CF_PID="$3"
EOF
}

stop_backend() {
  if [[ -f "$PID_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PID_FILE"

    for pid in "${CF_PID:-}" "${MOCAP_PID:-}" "${VRPN_PID:-}"; do
      if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
      fi
    done

    sleep 2

    for pid in "${CF_PID:-}" "${MOCAP_PID:-}" "${VRPN_PID:-}"; do
      if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
      fi
    done
  fi

  rm -f "$PID_FILE"
}

start_backend() {
  local vrpn_log="$LOG_DIR/vrpn.log"
  local mocap_log="$LOG_DIR/mocap.log"
  local cf_log="$LOG_DIR/cf_bridge.log"

  : > "$vrpn_log"
  : > "$mocap_log"
  : > "$cf_log"

  nohup bash -lc "
    source /opt/ros/$ROS_DISTRO/setup.bash
    source '$WS/install/setup.bash'
    exec ros2 run vrpn_mocap client_node --ros-args -p server:=$VRPN_SERVER -p port:=$VRPN_PORT
  " >"$vrpn_log" 2>&1 &
  local vrpn_pid=$!

  sleep 2

  nohup bash -lc "
    source /opt/ros/$ROS_DISTRO/setup.bash
    source '$WS/install/setup.bash'
    exec ros2 launch mocap_bridge_ros2 mocap_bridge.launch.py config_path:='$MOCAP_YAML'
  " >"$mocap_log" 2>&1 &
  local mocap_pid=$!

  sleep 2

  nohup bash -lc "
    source /opt/ros/$ROS_DISTRO/setup.bash
    source '$WS/install/setup.bash'
    exec ros2 launch cf_bridge cf_bridge.launch.py config_path:='$CF_YAML'
  " >"$cf_log" 2>&1 &
  local cf_pid=$!

  write_state_files "$vrpn_pid" "$mocap_pid" "$cf_pid"
}

wait_backend_ready() {
  local attempts=30
  local i

  for ((i=1; i<=attempts; i++)); do
    if backend_processes_alive && backend_ros_ready; then
      return 0
    fi
    sleep 1
  done

  echo "Backend did not become ready." >&2
  echo "Check logs in: $LOG_DIR" >&2
  return 1
}

run_sdk_algorithm() {
  source_ros_env
  # shellcheck disable=SC1090
  source "$VENV_DIR/bin/activate"
  cd "$REPO_ROOT"
  exec python3 "$ALGO"
}

main() {
  check_inputs
  check_workspace_ready

  echo "=== Experiment Launcher ==="
  echo "Repository:  $REPO_ROOT"
  echo "Algorithm:   $ALGO"
  echo "Workspace:   $WS"
  echo "Mocap YAML:  $MOCAP_YAML"
  echo "CF YAML:     $CF_YAML"
  echo "VRPN:        $VRPN_SERVER:$VRPN_PORT"
  echo "Probe topic: $FIRST_MOCAP_TOPIC"
  echo "Probe srv:   $TAKEOFF_SERVICE"
  echo "Logs dir:    $LOG_DIR"
  echo

  local restart_needed=0

  if ! backend_state_matches; then
    echo "Backend configuration differs from last run."
    restart_needed=1
  elif ! backend_processes_alive; then
    echo "Backend PID state is missing or stale."
    restart_needed=1
  elif ! backend_ros_ready; then
    echo "Backend processes exist but ROS probes failed."
    restart_needed=1
  fi

  if [[ $restart_needed -eq 1 ]]; then
    echo "Restarting backend..."
    stop_backend || true
    start_backend
    wait_backend_ready
    echo "Backend started successfully."
  else
    echo "Backend already running with matching configuration."
  fi

  echo "Launching SDK algorithm..."
  run_sdk_algorithm
}

main "$@"
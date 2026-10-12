#!/usr/bin/env bash
# Build the local MoveIt 2 package if needed, then open RViz planning.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[moveit] 需要已安装的 ROS 2 Humble（/opt/ros/humble）" >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WS"
colcon build --symlink-install --packages-select openarm_moveit_config
set +u
source "$WS/install/setup.bash"
set -u
LEFT="${OPENARM_LEFT_FOLLOWER_CAN:-can1}"
RIGHT="${OPENARM_RIGHT_FOLLOWER_CAN:-can0}"
USE_REAL="${OPENARM_MOVEIT_REAL:-false}"
case "${1:-}" in
  --real) USE_REAL=true ;;
  --sim) USE_REAL=false ;;
  "") ;;
  *)
    echo "[moveit] 用法：$0 [--sim|--real]" >&2
    exit 1
    ;;
esac
echo "[moveit] 模式=$([ "${USE_REAL}" = true ] && echo 实机 || echo 仿真) left=${LEFT} right=${RIGHT}"
if [[ "${USE_REAL}" == "true" ]]; then
  if ! ip link show "${LEFT}" >/dev/null 2>&1 || ! ip link show "${RIGHT}" >/dev/null 2>&1; then
    echo "[moveit] 实机需要 CAN 口 ${LEFT}/${RIGHT}，当前系统里没有。请先插好 FDCAN，再勾选急停启动。" >&2
    exit 1
  fi
fi
ERROR_LOG_DIR="${OPENARM_MOVEIT_ERROR_LOG_DIR:-$ROOT/recordings/moveit_gravity}"
GRAVITY_YAML="${OPENARM_GRAVITY_YAML:-$ROOT/config/gravity_comp.yaml}"
mkdir -p "$ERROR_LOG_DIR"
exec ros2 launch openarm_moveit_config demo.launch.py \
  "use_real:=${USE_REAL}" \
  "left_can:=${LEFT}" \
  "right_can:=${RIGHT}" \
  "error_log_dir:=${ERROR_LOG_DIR}" \
  "gravity_yaml:=${GRAVITY_YAML}"

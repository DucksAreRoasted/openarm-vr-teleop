#!/usr/bin/env bash
# Quest / OpenArm VR teleoperation.
#   ./run.sh                 simulation (default)
#   ./run.sh --real          both arms on SocketCAN
#   ./run.sh --no-bridge     do not start the Quest WebXR bridge
#   ./run.sh --print-paths   print resolved URDF and MuJoCo scene, then exit
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROL="${ROOT}/build/quest_cartesian_control"
BRIDGE="${ROOT}/quest/quest_bridge.py"
URDF="${OPENARM_TELEOP_URDF:-${ROOT}/assets/v1.urdf}"
# Scene resolution, in order:
#   1. OPENARM_MUJOCO_XML when it is set and non-empty
#   2. the historical machine path, when that file is still present
#   3. the scene shipped next to this script
LEGACY_XML="/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml"
REPO_XML="${ROOT}/models/OpenArm_V1/scene_table.xml"
if [[ -n "${OPENARM_MUJOCO_XML:-}" ]]; then
  XML="${OPENARM_MUJOCO_XML}"
elif [[ -f "${LEGACY_XML}" ]]; then
  XML="${LEGACY_XML}"
else
  XML="${REPO_XML}"
fi
if [[ -x "${ROOT}/.venv/bin/python" ]]; then
  PYTHON="${PYTHON:-${ROOT}/.venv/bin/python}"
else
  PYTHON="${PYTHON:-python3}"
fi

SIM=1
START_BRIDGE=1
PRINT_PATHS=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --real) SIM=0 ;;
    --sim) SIM=1 ;;
    --no-bridge) START_BRIDGE=0 ;;
    -h|--help)
      sed -n '2,7p' "$0"
      exit 0
      ;;
    --print-paths) PRINT_PATHS=1 ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done
if [[ "${PRINT_PATHS}" == "1" ]]; then
  printf 'ROOT=%s\nURDF=%s\nXML=%s\n' "${ROOT}" "${URDF}" "${XML}"
  exit 0
fi
if [[ ! -f "${URDF}" ]]; then
  echo "[vr-teleop][ERROR] URDF is missing: ${URDF}" >&2
  exit 2
fi
cmake -S "${ROOT}" -B "${ROOT}/build" -DPython3_EXECUTABLE="${PYTHON}"
cmake --build "${ROOT}/build" --target quest_cartesian_control -j"$(nproc)"

BRIDGE_PID=""
CONTROL_PID=""
PANEL_PID=""
cleanup() {
  if [[ -n "${CONTROL_PID}" ]] && kill -0 "${CONTROL_PID}" 2>/dev/null; then
    kill -INT "${CONTROL_PID}" 2>/dev/null || true
    wait "${CONTROL_PID}" || true
  fi
  for child_pid in "${BRIDGE_PID}" "${PANEL_PID}"; do
    if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
      kill -TERM "${child_pid}" 2>/dev/null || true
      wait "${child_pid}" || true
    fi
  done
}
trap cleanup EXIT INT TERM
if [[ "${START_BRIDGE}" == "1" ]]; then
  "${PYTHON}" "${BRIDGE}" --host 0.0.0.0 \
    --http-port "${OPENARM_QUEST_HTTP_PORT:-8443}" \
    --ws-port "${OPENARM_QUEST_WS_PORT:-8442}" \
    --udp-host "${OPENARM_QUEST_UDP_HOST:-127.0.0.1}" \
    --udp-left-port "${OPENARM_QUEST_PORT_LEFT:-47912}" \
    --udp-right-port "${OPENARM_QUEST_PORT_RIGHT:-47911}" &
  BRIDGE_PID=$!
fi
ARGS=(--bimanual --urdf "${URDF}"
  --quest-port-left "${OPENARM_QUEST_PORT_LEFT:-47912}"
  --quest-port-right "${OPENARM_QUEST_PORT_RIGHT:-47911}"
  --quest-ui-record-port "${OPENARM_QUEST_UI_RECORD_PORT:-47930}"
  --quest-ui-jog-port "${OPENARM_QUEST_UI_JOG_PORT:-47931}"
  --teleop-mode "${VR_TELEOP_MODE:-unitree_style}"
  --orientation-mapping "${VR_ORIENTATION:-absolute}")
if [[ -n "${OPENARM_QUEST_RECORD_DIR:-}" ]]; then
  ARGS+=(--quest-record-dir "${OPENARM_QUEST_RECORD_DIR}")
fi

if [[ "${SIM}" == "1" ]]; then
  if [[ ! -f "${XML}" ]]; then
    echo "[vr-teleop][ERROR] MuJoCo scene is missing: ${XML}" >&2
    exit 2
  fi
  ARGS+=(--sim --xml "${XML}")
else
  ARGS+=(--follower-can-left "${OPENARM_LEFT_FOLLOWER_CAN:-can1}"
    --follower-can-right "${OPENARM_RIGHT_FOLLOWER_CAN:-can0}")
fi
: "${OPENARM_QUEST_STARTUP_MOVE:=1}"
: "${OPENARM_QUEST_STARTUP_SEC:=4}"
: "${OPENARM_QUEST_STARTUP_TRANSITION_SEC:=5}"
if [[ -z "${OPENARM_QUEST_STARTUP_FILE+x}" ]]; then
  OPENARM_QUEST_STARTUP_FILE="${ROOT}/assets/steady_action_on_table.csv"
fi
if [[ "${OPENARM_QUEST_STARTUP_MOVE}" == "1" ]]; then
  ARGS+=(--startup-move)
  if [[ "${SIM}" == "1" ]]; then
    ARGS+=(--no-sim-table-ready)
  fi
  if [[ -n "${OPENARM_QUEST_STARTUP_FILE}" ]]; then
    if [[ ! -f "${OPENARM_QUEST_STARTUP_FILE}" ]]; then
      echo "[vr-teleop][ERROR] startup trajectory is missing: ${OPENARM_QUEST_STARTUP_FILE}" >&2
      exit 2
    fi
    ARGS+=(--startup-file "${OPENARM_QUEST_STARTUP_FILE}")
  fi
  ARGS+=(--startup-move-sec "${OPENARM_QUEST_STARTUP_SEC}"
    --startup-transition-sec "${OPENARM_QUEST_STARTUP_TRANSITION_SEC}")
fi

cd "${ROOT}"
export OPENARM_LEROBOT_PYTHON="${PYTHON}"
if [[ "${OPENARM_RECORD_PANEL:-1}" != "0" ]]; then
  "${PYTHON}" "${ROOT}/script/record_panel.py" \
    --port "${OPENARM_RECORD_PANEL_PORT:-8765}" \
    --udp-port "${OPENARM_QUEST_UI_RECORD_PORT:-47930}" &
  PANEL_PID=$!
fi
"${CONTROL}" "${ARGS[@]}" &
CONTROL_PID=$!
wait "${CONTROL_PID}"

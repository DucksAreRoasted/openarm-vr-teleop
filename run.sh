#!/usr/bin/env bash
# Quest / OpenArm VR teleoperation.
#   ./run.sh                 simulation (default)
#   ./run.sh --real          both arms on SocketCAN
#   ./run.sh --skeleton --sim shoulder/elbow body tracking + controller wrist
#   ./run.sh --mirror        right hand-guided, left same-direction follower (real CAN)
#   ./run.sh --no-bridge     do not start the Quest WebXR bridge
#   ./run.sh --ker --sim     KER USB leader -> V1 simulated follower
#   ./run.sh --ker --real    calibrated KER -> V1 real follower
#   ./run.sh --ker-demo      simulation with scripted KER input (no USB device)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROL="${ROOT}/build/quest_cartesian_control"
BRIDGE="${ROOT}/quest/quest_bridge.py"
URDF="${OPENARM_TELEOP_URDF:-${ROOT}/assets/v1.urdf}"
XML="${OPENARM_MUJOCO_XML:-/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml}"
if [[ -x "${ROOT}/.venv/bin/python" ]]; then
  PYTHON="${PYTHON:-${ROOT}/.venv/bin/python}"
else
  PYTHON="${PYTHON:-python3}"
fi

SIM=1
START_BRIDGE=1
MIRROR=0
KER=0
KER_DEMO=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --real) SIM=0 ;;
    --sim) SIM=1 ;;
    --skeleton) export VR_TELEOP_MODE=skeleton_proximal ;;
    --mirror) MIRROR=1; SIM=0; START_BRIDGE=0 ;;
    --ker) KER=1 ;;
    --ker-demo) KER=1; KER_DEMO=1 ;;
    --no-bridge) START_BRIDGE=0 ;;
    -h|--help)
      sed -n '2,10p' "$0"
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done
if [[ "${VR_TELEOP_MODE:-}" == "skeleton_proximal" ]]; then
  export VR_TELEOP_MODE
fi
if [[ "${MIRROR}" == "1" ]]; then
  START_BRIDGE=0
fi
if [[ "${KER}" == "1" ]]; then
  START_BRIDGE=0
  if [[ "${MIRROR}" == "1" || ( "${KER_DEMO}" == "1" && "${SIM}" == "0" ) ]]; then
    echo "[ker][ERROR] KER cannot be combined with mirror; demo requires simulation" >&2
    exit 2
  fi
  export OPENARM_KER_CONFIG="${OPENARM_KER_CONFIG:-${ROOT}/config/ker.yaml}"
  KER_PYTHON="${OPENARM_KER_PYTHON:-${PYTHON}}"
  if [[ -z "${OPENARM_KER_PYTHON:-}" && -x "${ROOT}/.venv-ker/bin/python" ]]; then
    KER_PYTHON="${ROOT}/.venv-ker/bin/python"
  fi
  KER_ARGS=(--config "${OPENARM_KER_CONFIG}" --udp-port "${OPENARM_KER_PORT:-47932}"
    --transport "${OPENARM_KER_TRANSPORT:-usb}" --serial-port "${OPENARM_KER_SERIAL_PORT:-/dev/ttyACM0}")
  if [[ "${SIM}" == "0" ]]; then KER_ARGS+=(--real); fi
  if [[ "${KER_DEMO}" == "1" ]]; then KER_ARGS+=(--demo); fi
  "${KER_PYTHON}" "${ROOT}/script/ker_bridge.py" "${KER_ARGS[@]}" --check
  if [[ "${KER_DEMO}" != "1" ]]; then
    # Fail before compilation / follower startup if USB access or handshake fails.
    "${KER_PYTHON}" "${ROOT}/script/ker_bridge.py" "${KER_ARGS[@]}" --probe
  fi
fi

if [[ ! -f "${ROOT}/CMakeLists.txt" ]]; then
  echo "[vr-teleop][ERROR] CMakeLists.txt is missing" >&2
  exit 2
fi
cmake -S "${ROOT}" -B "${ROOT}/build" -DPython3_EXECUTABLE="${PYTHON}"
cmake --build "${ROOT}/build" --target quest_cartesian_control -j"$(nproc)"
if [[ ! -f "${URDF}" ]]; then
  echo "[vr-teleop][ERROR] URDF is missing: ${URDF}" >&2
  exit 2
fi

BRIDGE_PID=""
KER_PID=""
CONTROL_PID=""
PANEL_PID=""
cleanup() {
  if [[ -n "${CONTROL_PID}" ]] && kill -0 "${CONTROL_PID}" 2>/dev/null; then
    kill -INT "${CONTROL_PID}" 2>/dev/null || true
    wait "${CONTROL_PID}" || true
  fi
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -TERM "${BRIDGE_PID}" 2>/dev/null || true
    wait "${BRIDGE_PID}" || true
  fi
  if [[ -n "${KER_PID}" ]] && kill -0 "${KER_PID}" 2>/dev/null; then
    kill -TERM "${KER_PID}" 2>/dev/null || true
    wait "${KER_PID}" || true
  fi
  if [[ -n "${PANEL_PID}" ]] && kill -0 "${PANEL_PID}" 2>/dev/null; then
    kill -TERM "${PANEL_PID}" 2>/dev/null || true
    wait "${PANEL_PID}" || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${START_BRIDGE}" == "1" ]]; then
  # Headset page stays on 0.0.0.0. UDP to the control program is loopback unless
  # OPENARM_QUEST_UDP_HOST is set (0.0.0.0 or a specific IPv4). The control
  # process reads that same variable; this flag only sets the bridge target.
  "${PYTHON}" "${BRIDGE}" --host 0.0.0.0 \
    --http-port "${OPENARM_QUEST_HTTP_PORT:-8443}" \
    --ws-port "${OPENARM_QUEST_WS_PORT:-8442}" \
    --udp-host "${OPENARM_QUEST_UDP_HOST:-127.0.0.1}" \
    --udp-left-port "${OPENARM_QUEST_PORT_LEFT:-47912}" \
    --udp-right-port "${OPENARM_QUEST_PORT_RIGHT:-47911}" &
  BRIDGE_PID=$!
fi

ARGS=(--bimanual --urdf "${URDF}"
  --skeleton-pose-port "${OPENARM_SKELETON_POSE_PORT:-47951}"
  --quest-port-left "${OPENARM_QUEST_PORT_LEFT:-47912}"
  --quest-port-right "${OPENARM_QUEST_PORT_RIGHT:-47911}"
  --quest-ui-record-port "${OPENARM_QUEST_UI_RECORD_PORT:-47930}"
  --quest-ui-jog-port "${OPENARM_QUEST_UI_JOG_PORT:-47931}"
  --teleop-mode "${VR_TELEOP_MODE:-human_mimic}"
  --orientation-mapping "${VR_ORIENTATION:-absolute}")
if [[ "${MIRROR}" == "1" ]]; then
  ARGS+=(--mirror-teach)
  export OPENARM_ROOT="${ROOT}"
  export OPENARM_GRAVITY_YAML="${OPENARM_GRAVITY_YAML:-${ROOT}/config/gravity_comp.yaml}"
  export OPENARM_LEROBOT_TASK="${OPENARM_LEROBOT_TASK:-right hand-guided same-direction teaching}"
fi
if [[ "${KER}" == "1" ]]; then
  ARGS+=(--ker --ker-port "${OPENARM_KER_PORT:-47932}")
  export OPENARM_LEROBOT_TASK="${OPENARM_LEROBOT_TASK:-KER V1 bimanual teaching}"
fi
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
  : "${OPENARM_QUEST_STARTUP_MOVE:=1}"
  : "${OPENARM_QUEST_STARTUP_SEC:=4}"
  : "${OPENARM_QUEST_STARTUP_TRANSITION_SEC:=5}"
  if [[ -z "${OPENARM_QUEST_STARTUP_FILE+x}" ]]; then
    OPENARM_QUEST_STARTUP_FILE="${ROOT}/assets/steady_action_on_table.csv"
  fi
  if [[ "${MIRROR}" != "1" && "${KER}" != "1" && "${OPENARM_QUEST_STARTUP_MOVE}" == "1" ]]; then
    ARGS+=(--startup-move)
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
fi

cd "${ROOT}"
export OPENARM_LEROBOT_PYTHON="${PYTHON}"
if [[ "${OPENARM_RECORD_PANEL:-1}" != "0" ]]; then
  "${PYTHON}" "${ROOT}/script/record_panel.py" \
    --port "${OPENARM_RECORD_PANEL_PORT:-8765}" \
    --udp-port "${OPENARM_QUEST_UI_RECORD_PORT:-47930}" &
  PANEL_PID=$!
fi
if [[ "${KER}" == "1" ]]; then
  "${KER_PYTHON}" "${ROOT}/script/ker_bridge.py" "${KER_ARGS[@]}" &
  KER_PID=$!
fi
"${CONTROL}" "${ARGS[@]}" &
CONTROL_PID=$!
if [[ "${KER}" == "1" ]]; then
  # A USB bridge failure must also stop the servo; a servo exit closes the USB stream.
  wait -n "${CONTROL_PID}" "${KER_PID}"
else
  wait "${CONTROL_PID}"
fi

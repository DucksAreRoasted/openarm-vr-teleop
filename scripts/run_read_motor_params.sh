#!/usr/bin/env bash
# Read Damiao motor parameters from both arms (no enable / no motion).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ ! -f build/openarm_read_motor_params ]]; then
  cmake -S . -B build
fi
cmake --build build --target openarm_read_motor_params -j"$(nproc)"
export OPENARM_ROOT="$ROOT"
export OPENARM_LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-can1}"
export OPENARM_RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-can0}"
export OPENARM_MOTOR_PARAM_DIR="${OPENARM_MOTOR_PARAM_DIR:-$ROOT/recordings/motor_params}"
mkdir -p "$OPENARM_MOTOR_PARAM_DIR"
echo "[motor-params] left=${OPENARM_LEFT_FOLLOWER_CAN} right=${OPENARM_RIGHT_FOLLOWER_CAN}"
exec "$ROOT/build/openarm_read_motor_params"

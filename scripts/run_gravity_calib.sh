#!/usr/bin/env bash
# Build and run automatic gravity calibration trajectory (logs only).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ ! -f build/openarm_gravity_calib ]]; then
  cmake -S . -B build
fi
cmake --build build --target openarm_gravity_calib -j"$(nproc)"
export OPENARM_ROOT="$ROOT"
export OPENARM_URDF="${OPENARM_URDF:-$ROOT/assets/v1.urdf}"
export OPENARM_GRAVITY_YAML="${OPENARM_GRAVITY_YAML:-$ROOT/config/gravity_comp.yaml}"
export OPENARM_ZERO_G_LOG_DIR="${OPENARM_ZERO_G_LOG_DIR:-$ROOT/recordings/gravity}"
export OPENARM_LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-can1}"
export OPENARM_RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-can0}"
mkdir -p "$OPENARM_ZERO_G_LOG_DIR"
echo "[calib] left=${OPENARM_LEFT_FOLLOWER_CAN} right=${OPENARM_RIGHT_FOLLOWER_CAN}"
exec "$ROOT/build/openarm_gravity_calib" "$@"

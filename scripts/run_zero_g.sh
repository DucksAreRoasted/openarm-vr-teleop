#!/usr/bin/env bash
# Build and run zero-gravity feel mode (gravity FF only, logs for tuning).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ ! -f build/openarm_zero_g ]]; then
  cmake -S . -B build
fi
cmake --build build --target openarm_zero_g -j"$(nproc)"
export OPENARM_ROOT="$ROOT"
export OPENARM_URDF="${OPENARM_URDF:-$ROOT/assets/v1.urdf}"
export OPENARM_GRAVITY_YAML="${OPENARM_GRAVITY_YAML:-$ROOT/config/gravity_comp.yaml}"
export OPENARM_ZERO_G_LOG_DIR="${OPENARM_ZERO_G_LOG_DIR:-$ROOT/recordings/gravity}"
export OPENARM_LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-can1}"
export OPENARM_RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-can0}"
mkdir -p "$OPENARM_ZERO_G_LOG_DIR"
echo "[zero-g] left=${OPENARM_LEFT_FOLLOWER_CAN} right=${OPENARM_RIGHT_FOLLOWER_CAN}"
exec "$ROOT/build/openarm_zero_g"

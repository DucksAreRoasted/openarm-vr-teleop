#!/usr/bin/env bash
# Open the teleop window: simulation or real OpenArm.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "${ROOT}/.venv/bin/python" ]]; then
  PYTHON="${PYTHON:-${ROOT}/.venv/bin/python}"
else
  PYTHON="${PYTHON:-python3}"
fi
exec "${PYTHON}" "${ROOT}/script/teleop_app.py"

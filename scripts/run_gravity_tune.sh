#!/usr/bin/env bash
# Eight sliders: shared link mass scales + gripper motor kg.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -x "${ROOT}/.venv/bin/python" ]]; then
  PYTHON="${PYTHON:-${ROOT}/.venv/bin/python}"
else
  PYTHON="${PYTHON:-python3}"
fi
exec "${PYTHON}" "${ROOT}/script/gravity_tune_ui.py"

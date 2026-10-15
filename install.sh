#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-python3}"

"${PYTHON}" -m venv "${ROOT}/.venv"
"${ROOT}/.venv/bin/python" -m pip install --upgrade pip
"${ROOT}/.venv/bin/python" -m pip install -r "${ROOT}/requirements.txt"
# Optional: official OpenArm Dataset package (convert / validate). Soft-fail on flaky mirrors.
if ! "${ROOT}/.venv/bin/python" -c "import openarm_dataset" 2>/dev/null; then
  "${ROOT}/.venv/bin/python" -m pip install --no-deps openarm-dataset \
    || echo "Note: openarm-dataset not installed; local CSV→LeRobot fallback still works."
fi
echo "Installed Python packages in ${ROOT}/.venv"

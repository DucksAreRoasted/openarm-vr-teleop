#!/usr/bin/env bash
# Keep the official Python 3.11+ KER SDK separate from the Qt / control environment.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if command -v uv >/dev/null 2>&1; then
  uv venv "${ROOT}/.venv-ker" --allow-existing --python "${KER_PYTHON_VERSION:-3.12}"
  uv pip install --python "${ROOT}/.venv-ker/bin/python" -r "${ROOT}/requirements-ker.txt"
else
  KER_SETUP_PYTHON="${KER_SETUP_PYTHON:-python3.12}"
  "${KER_SETUP_PYTHON}" -c 'import sys; assert sys.version_info >= (3, 11), "KER SDK needs Python 3.11+"'
  "${KER_SETUP_PYTHON}" -m venv "${ROOT}/.venv-ker"
  "${ROOT}/.venv-ker/bin/python" -m pip install -r "${ROOT}/requirements-ker.txt"
fi
"${ROOT}/.venv-ker/bin/python" -c 'from openarm_ker.ker_stream import KERStream; import usb.core, serial; print("KER SDK ready")'

#!/usr/bin/env python3
"""OpenArm desktop control — Qt entry (./app.sh)."""

from __future__ import annotations

import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent
if str(SCRIPT) not in sys.path:
    sys.path.insert(0, str(SCRIPT))

from openarm_app import main

if __name__ == "__main__":
    main()

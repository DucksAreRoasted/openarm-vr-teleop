#!/usr/bin/env python3
"""Joint fine-tune UI — Qt dialog (standalone or from main app)."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "script"
if str(SCRIPT) not in sys.path:
    sys.path.insert(0, str(SCRIPT))


def main() -> None:
    from PySide6.QtWidgets import QApplication

    from openarm_app.gravity_tune_dialog import GravityTuneDialog
    from openarm_app.theme import APP_STYLE

    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLE)
    dlg = GravityTuneDialog()
    dlg.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

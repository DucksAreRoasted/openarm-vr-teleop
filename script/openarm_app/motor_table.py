"""Parse motor-parameter CSV into rows for the QML motor page."""

from __future__ import annotations

import csv
from pathlib import Path

from PySide6.QtCore import QAbstractListModel, QModelIndex, Qt

_MODES = {1: "MIT", 2: "位置速度", 3: "速度", 4: "力位"}


def _missing(number: float) -> bool:
    return -1.5 < number < -0.5


def _format_number(text: str) -> str:
    try:
        number = float(text)
    except (TypeError, ValueError):
        return "—"
    if _missing(number):
        return "—"
    if abs(number) >= 1 and abs(number - round(number)) < 1e-4 and abs(number) < 1e6:
        return str(int(round(number)))
    return f"{number:.4g}"


def _format_mode(text: str) -> str:
    try:
        number = float(text)
    except (TypeError, ValueError):
        return "—"
    if _missing(number):
        return "—"
    return _MODES.get(int(round(number)), _format_number(text))


def load_motor_rows(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        for record in csv.DictReader(handle):
            side = (record.get("side") or "").strip().lower()
            rows.append(
                {
                    "side": side,
                    "joint": record.get("joint") or "",
                    "typeName": record.get("type") or "—",
                    "mode": _format_mode(record.get("CTRL_MODE") or ""),
                    "kt": _format_number(record.get("KT_Value") or ""),
                    "gear": _format_number(record.get("Gr") or ""),
                    "poles": _format_number(record.get("NPP") or ""),
                    "resistance": _format_number(record.get("Rs") or ""),
                    "inductance": _format_number(record.get("LS") or ""),
                    "flux": _format_number(record.get("Flux") or ""),
                    "inertia": _format_number(record.get("Inertia") or ""),
                    "damp": _format_number(record.get("Damp") or ""),
                    "pmax": _format_number(record.get("PMAX") or ""),
                    "vmax": _format_number(record.get("VMAX") or ""),
                    "tmax": _format_number(record.get("TMAX") or ""),
                }
            )
    return rows


class MotorTable(QAbstractListModel):
    JointRole = Qt.ItemDataRole.UserRole + 1
    TypeRole = Qt.ItemDataRole.UserRole + 2
    ModeRole = Qt.ItemDataRole.UserRole + 3
    KtRole = Qt.ItemDataRole.UserRole + 4
    GearRole = Qt.ItemDataRole.UserRole + 5
    PolesRole = Qt.ItemDataRole.UserRole + 6
    ResistanceRole = Qt.ItemDataRole.UserRole + 7
    InductanceRole = Qt.ItemDataRole.UserRole + 8
    FluxRole = Qt.ItemDataRole.UserRole + 9
    InertiaRole = Qt.ItemDataRole.UserRole + 10
    DampRole = Qt.ItemDataRole.UserRole + 11
    PmaxRole = Qt.ItemDataRole.UserRole + 12
    VmaxRole = Qt.ItemDataRole.UserRole + 13
    TmaxRole = Qt.ItemDataRole.UserRole + 14

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._all: list[dict[str, str]] = []
        self._rows: list[dict[str, str]] = []
        self.side = "left"

    def roleNames(self):  # noqa: N802
        return {
            self.JointRole: b"joint",
            self.TypeRole: b"typeName",
            self.ModeRole: b"modeName",
            self.KtRole: b"kt",
            self.GearRole: b"gear",
            self.PolesRole: b"poles",
            self.ResistanceRole: b"resistance",
            self.InductanceRole: b"inductance",
            self.FluxRole: b"flux",
            self.InertiaRole: b"inertia",
            self.DampRole: b"damp",
            self.PmaxRole: b"pmax",
            self.VmaxRole: b"vmax",
            self.TmaxRole: b"tmax",
        }

    def rowCount(self, parent=QModelIndex()):  # noqa: N802
        if parent.isValid():
            return 0
        return len(self._rows)

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):  # noqa: N802
        if not index.isValid() or not (0 <= index.row() < len(self._rows)):
            return None
        row = self._rows[index.row()]
        return {
            self.JointRole: row["joint"],
            self.TypeRole: row["typeName"],
            self.ModeRole: row["mode"],
            self.KtRole: row["kt"],
            self.GearRole: row["gear"],
            self.PolesRole: row["poles"],
            self.ResistanceRole: row["resistance"],
            self.InductanceRole: row["inductance"],
            self.FluxRole: row["flux"],
            self.InertiaRole: row["inertia"],
            self.DampRole: row["damp"],
            self.PmaxRole: row["pmax"],
            self.VmaxRole: row["vmax"],
            self.TmaxRole: row["tmax"],
        }.get(role)

    def replace(self, rows: list[dict[str, str]]) -> None:
        self._all = rows
        self._apply()

    def set_side(self, side: str) -> None:
        if side not in ("left", "right") or side == self.side:
            return
        self.side = side
        self._apply()

    def _apply(self) -> None:
        shown = [row for row in self._all if row["side"] == self.side]
        self.beginResetModel()
        self._rows = shown
        self.endResetModel()

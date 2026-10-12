"""Shared Qt styling and window geometry helpers."""

from __future__ import annotations

from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import QWidget

APP_STYLE = """
QWidget {
    font-family: "Noto Sans CJK SC", "Source Han Sans SC", "PingFang SC",
                 "Microsoft YaHei", "Noto Sans", "DejaVu Sans", sans-serif;
    font-size: 15px;
    color: #1f2933;
}
QMainWindow, QDialog {
    background: #eef2f6;
}
QLabel#HeroTitle {
    font-size: 24px;
    font-weight: 700;
    color: #102a43;
    letter-spacing: 0.2px;
    padding-bottom: 2px;
}
QLabel#HeroSub, QLabel#Hint {
    color: #627d98;
    font-size: 14px;
}
QLabel#StatusWarn {
    color: #b45309;
    font-weight: 600;
    font-size: 15px;
}
/* Title sits in the top margin so it does not cross the border line. */
QGroupBox {
    background: #ffffff;
    border: 1px solid #d9e2ec;
    border-radius: 10px;
    margin-top: 18px;
    padding: 18px 14px 14px 14px;
    font-size: 15px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 14px;
    top: 2px;
    padding: 0 8px;
    color: #243b53;
    background-color: #eef2f6;
}
QLineEdit, QComboBox, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox {
    background: #f8fafc;
    border: 1px solid #cbd2d9;
    border-radius: 6px;
    padding: 7px 10px;
    min-height: 22px;
    font-size: 15px;
    selection-background-color: #0e7490;
    selection-color: white;
}
QLineEdit:focus, QComboBox:focus, QTextEdit:focus, QDoubleSpinBox:focus {
    border: 1px solid #0e7490;
    background: #ffffff;
}
QComboBox::drop-down {
    border: none;
    width: 26px;
}
QPushButton {
    background: #e4ebf2;
    border: 1px solid #bcccdc;
    border-radius: 7px;
    padding: 9px 16px;
    min-height: 34px;
    font-size: 15px;
}
QPushButton:hover {
    background: #d9e2ec;
}
QPushButton:pressed {
    background: #bcccdc;
}
QPushButton:disabled {
    color: #9fb3c8;
    background: #f0f4f8;
    border-color: #e0e6ed;
}
QPushButton#Primary {
    background: #0e7490;
    color: white;
    border: 1px solid #0e7490;
    font-weight: 700;
}
QPushButton#Primary:hover {
    background: #0f766e;
    border-color: #0f766e;
}
QPushButton#Primary:disabled {
    background: #9fb3c8;
    border-color: #9fb3c8;
    color: #f0f4f8;
}
QPushButton#Danger {
    background: #fff5f5;
    color: #9b1c1c;
    border: 1px solid #f5c2c2;
}
QPushButton#Danger:hover {
    background: #ffe3e3;
}
QPushButton#Accent {
    background: #ecfeff;
    color: #155e75;
    border: 1px solid #a5f3fc;
    font-weight: 600;
}
QRadioButton, QCheckBox {
    spacing: 10px;
    font-size: 15px;
}
QRadioButton::indicator, QCheckBox::indicator {
    width: 18px;
    height: 18px;
}
QSlider::groove:horizontal {
    height: 8px;
    background: #d9e2ec;
    border-radius: 4px;
}
QSlider::handle:horizontal {
    width: 18px;
    margin: -6px 0;
    border-radius: 9px;
    background: #0e7490;
}
QSlider::sub-page:horizontal {
    background: #67e8f9;
    border-radius: 4px;
}
QScrollArea {
    border: none;
    background: transparent;
}
QTextEdit {
    background: #0b1220;
    color: #d9e2ec;
    border: 1px solid #243b53;
    border-radius: 8px;
    font-family: "JetBrains Mono", "Cascadia Code", "Consolas", monospace;
    font-size: 13px;
    padding: 8px;
}
"""


def fit_window(
    widget: QWidget,
    *,
    preferred_w: int,
    preferred_h: int,
    min_w: int,
    min_h: int,
    max_screen_ratio: float = 0.88,
) -> None:
    """Size and center a window within the available screen area."""
    screen = QGuiApplication.primaryScreen()
    if screen is None:
        widget.resize(preferred_w, preferred_h)
        return
    avail = screen.availableGeometry()
    w = min(preferred_w, int(avail.width() * max_screen_ratio))
    h = min(preferred_h, int(avail.height() * max_screen_ratio))
    w = max(w, min(min_w, avail.width() - 40))
    h = max(h, min(min_h, avail.height() - 40))
    widget.setMinimumSize(min(min_w, w), min(min_h, h))
    widget.resize(w, h)
    widget.move(
        avail.x() + (avail.width() - w) // 2,
        avail.y() + (avail.height() - h) // 2,
    )

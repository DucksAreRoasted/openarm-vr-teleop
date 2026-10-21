"""QML console controller for OpenArm teleop / MoveIt / zero-g control."""

from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time

from PySide6.QtCore import (
    Property,
    QObject,
    QSettings,
    Qt,
    QTimer,
    QUrl,
    Signal,
    Slot,
)
from PySide6.QtNetwork import QHostAddress, QUdpSocket
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtWidgets import QApplication, QInputDialog, QLineEdit, QMessageBox

from lerobot_cameras import default_config, list_usb_cameras, load_specs, save_specs

from .data_catalog import DataCatalog
from .gravity_identify import (
    friction_collect_command,
    friction_log_path,
    friction_summary,
    latest_two_sided_log,
    run_friction_write,
    run_identification,
    summary as identify_summary,
)
from .gravity_urdf_seed import install_urdf_gravity_seed
from .log_panel import LogPanel
from .motor_table import MotorTable, load_motor_rows
from .joint_angles import JointAngles
from .replay_dialog import ReplayBridge, ReplayDialog
from .services import (
    HTTP_PORT,
    ROOT,
    RUN,
    camera_view_open,
    configure_can,
    local_ip,
    quest_udp_host,
    resolve_gravity_yaml,
    run_ros2,
    send_record,
    sudo_is_cached,
)
from .theme import APP_STYLE

# Quest 回包必须立刻返回：quest_bridge 对 UDP 47934 只等约 0.8 秒，不能弹模态框。
# 界面用中文。传给 MoveIt 的仍是 P0 / P1 / P2 / P3 / drag。
IMPEDANCE_LABELS = {
    "P0": "正常刚度",
    "P1": "中等刚度",
    "P2": "偏低刚度",
    "P3": "接触柔顺",
    "drag": "手拖",
}
IMPEDANCE_TIPS = {
    "P0": "和原来的 MoveIt 一样硬，适合走轨迹。",
    "P1": "还跟得上轨迹，轻轻推会让一点。",
    "P2": "推一下会明显让开。",
    "P3": "碰到东西会让。",
    "drag": "几乎不往回拉，可以大幅掰动。不作为急停。",
}
QUEST_NOT_READY_MESSAGE = "遥操尚未就绪，暂时不能录制。"
UNDO_SECONDS = 10.0
DISCARD_HINT = "这一条已移到回收站。10 秒内再长按左摇杆或点「撤销丢弃」可恢复。"
UNDO_EXPIRED_HINT = "撤销时间已过，这一条留在回收站。再次开始会用同一个序号。"
RESTORED_HINT = "已恢复这一条。可以标注、重录，或开始下一条。"
_QUEST_RUNNING_RE = re.compile(r"(?m)^\[quest-cartesian\] (?:bimanual )?running\.\s*$")
_QUEST_PROFILES = (("宇树参考", "unitree_style"),)
_MODES = ("sim", "real", "moveit", "zerog")
_MODE_LABELS = {"sim": "仿真遥操", "real": "真机遥操", "moveit": "MoveIt", "zerog": "零重力"}
_RECORD_LABELS = {
    "recording": "录制中",
    "pending": "待标注",
    "ready": "可以录制",
    "idle": "未就绪",
}
_CAMERA_KEYS = ("head", "wrist_left", "wrist_right")
_PAGES = ("teleop", "capture", "gravity", "data", "replay", "motors", "teach")


class WorkerBridge(QObject):
    finished_ok = Signal()
    finished_fail = Signal(str)
    calib_done = Signal(bool, str)
    calib_log = Signal(str)
    identify_done = Signal(bool, bool, str)
    friction_done = Signal(bool, bool, str)
    motor_done = Signal(bool, str)
    idle = Signal(str)
    stop_stuck = Signal(str)
    notice = Signal(str)
    idle_refresh = Signal()


class _TextProxy:
    def __init__(self, getter, setter=None):
        self._getter = getter
        self._setter = setter

    def text(self):
        return self._getter()

    def setText(self, value):  # noqa: N802
        if self._setter is not None:
            self._setter(value)

    def setStyleSheet(self, _value):  # noqa: N802
        return None


class _ButtonProxy:
    def __init__(self, enabled, click=None, text=None, *, allow_override=False):
        self._enabled = enabled
        self._click = click
        self._text = text
        self._override = None
        self._allow_override = allow_override

    def isEnabled(self):  # noqa: N802
        if self._allow_override and self._override is not None:
            return self._override
        return bool(self._enabled())

    def setEnabled(self, value):  # noqa: N802
        if self._allow_override:
            self._override = bool(value)

    def click(self):
        if self._click is not None:
            self._click()

    def text(self):
        return self._text() if self._text is not None else ""


class _RadioProxy:
    def __init__(self, console: "ConsoleController", mode: str):
        self._console = console
        self._mode = mode

    def setChecked(self, checked):  # noqa: N802
        if checked:
            self._console._force_mode(self._mode)

    def isChecked(self):  # noqa: N802
        return self._console._mode_name == self._mode

    def isEnabled(self):  # noqa: N802
        return not self._console._running()


class _SafetyProxy:
    def __init__(self, console: "ConsoleController"):
        self._console = console

    def setChecked(self, checked):  # noqa: N802
        self._console.setSafetyChecked(bool(checked))

    def isChecked(self):  # noqa: N802
        return self._console._safety


class _ProfileProxy:
    def __init__(self, console: "ConsoleController"):
        self._console = console

    def currentData(self):  # noqa: N802
        return self._console._quest_profile

    def count(self):
        return len(_QUEST_PROFILES)

    def itemData(self, index):  # noqa: N802
        return _QUEST_PROFILES[index][1]

    def findData(self, data):  # noqa: N802
        for index, (_title, value) in enumerate(_QUEST_PROFILES):
            if value == data:
                return index
        return -1

    def setCurrentIndex(self, index):  # noqa: N802
        self._console._quest_profile = _QUEST_PROFILES[index][1]
        self._console.questProfileChanged.emit()

    def isEnabled(self):  # noqa: N802
        return self._console._mode_name in ("sim", "real") and not self._console._running()


class ConsoleController(QObject):
    """Process, recording, and gravity logic. QML binds to the properties below."""

    modeChanged = Signal()
    uiChanged = Signal()
    statusTextChanged = Signal()
    statusToneChanged = Signal()
    recordHintChanged = Signal()
    recordStateChanged = Signal()
    questProfileChanged = Signal()
    safetyCheckedChanged = Signal()
    batchNameChanged = Signal()
    canChanged = Signal()
    camerasChanged = Signal()
    cameraStatusChanged = Signal()
    pageChanged = Signal()
    logExpandedChanged = Signal()
    logHeightChanged = Signal()
    logCountsChanged = Signal()
    logDetachedChanged = Signal()
    closeRequested = Signal()
    motorInfoChanged = Signal()
    impedanceChanged = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.proc: subprocess.Popen[bytes] | None = None
        self.log_path = Path(tempfile.gettempdir()) / "openarm_vr_teleop_app.log"
        self.log_handle = None
        self._awaiting_label = False
        self._can_redo = False
        self._recording = False
        self._quest_ready = False
        self._undo_until = 0.0
        self._undo_awaiting = False
        self._usb_by_label: dict = {}
        self._camera_choice = {name: "（不选）" for name in _CAMERA_KEYS}
        self._mode_name = "sim"
        self._status_text = "未启动"
        self._status_tone = "idle"
        self._record_hint = "启动后可录。停下来再标成功、失败，或重录。"
        self._record_state = "idle"
        self._safety = False
        self._batch_name = ""
        self._left_can = os.environ.get("OPENARM_LEFT_FOLLOWER_CAN", "can1")
        self._right_can = os.environ.get("OPENARM_RIGHT_FOLLOWER_CAN", "can0")
        self._bitrate = os.environ.get("OPENARM_CAN_BITRATE", "1000000")
        self._dbitrate = os.environ.get("OPENARM_CAN_DBITRATE", "5000000")
        self._quest_profile = "unitree_style"
        self._quest_profile_requested = self._quest_profile
        self._impedance_profile = "P0"
        self._moveit_profile_requested = "P0"
        self._camera_status = "刷新后为头部 / 左腕 / 右腕选择 USB 口"
        self._page = "teleop"
        self._log_expanded = False
        self._log_height = 260
        self._log_detached = False
        self._viewport_height = 0
        self._stop_pending = False
        self._configuration_busy = False
        self._moveit_real = False
        self._impedance_busy = False
        self._calib_proc: subprocess.Popen[str] | None = None
        from .log_panel import IncrementalLogReader
        self._log_reader = IncrementalLogReader()
        self._identify_log: Path | None = None
        self._friction_log: Path | None = None
        self._log_ready_tail = ""

        self.bridge = WorkerBridge()
        self.bridge.finished_ok.connect(self._started)
        self.bridge.finished_fail.connect(self._fail)
        self.bridge.calib_done.connect(self._auto_calib_done)
        self.bridge.calib_log.connect(self._calib_log_line)
        self.bridge.identify_done.connect(self._identify_done)
        self.bridge.friction_done.connect(self._friction_done)
        self.bridge.motor_done.connect(self._read_motor_params_done)
        self.bridge.idle.connect(self._idle)
        self.bridge.stop_stuck.connect(self._stop_stuck)
        self.bridge.notice.connect(self._notice)
        self.bridge.idle_refresh.connect(self._apply_impedance_state)

        self._settings = QSettings("OpenArm", "TeleopConsole")
        saved_page = str(self._settings.value("page", "teleop") or "teleop")
        self._page = saved_page if saved_page in _PAGES else "teleop"
        self._log_height = max(120, int(self._settings.value("log/height", 260) or 260))
        archive = Path(tempfile.gettempdir()) / (
            f"openarm_console_{datetime.now():%Y%m%d_%H%M%S_%f}_{os.getpid()}.log"
        )
        self.log_panel = LogPanel(None, settings=self._settings, archive_path=archive)
        self.log = self.log_panel.view
        self._log_expanded = self.log_panel.toggle.isChecked()
        self.log_panel.countsChanged.connect(self.logCountsChanged.emit)
        self.log_panel.fatal_received.connect(self._log_fatal)
        self.log_panel.pause.toggled.connect(self.logCountsChanged.emit)
        self._catalog = DataCatalog(self)

        self.status = _TextProxy(lambda: self._status_text, self._set_status)
        self.record_hint = _TextProxy(lambda: self._record_hint, self._set_record_hint)
        self.batch_edit = _TextProxy(lambda: self._batch_name, self._set_batch_name)
        self.rec_start = _ButtonProxy(self._can_record_start)
        self.rec_stop = _ButtonProxy(self._can_record_stop)
        self.rec_ok = _ButtonProxy(self._can_label)
        self.rec_bad = _ButtonProxy(self._can_label)
        self.rec_redo = _ButtonProxy(self._can_press_redo, self._on_redo_button, self._redo_label)
        self.btn_start = _ButtonProxy(self._can_press_start, self._start, allow_override=True)
        self.btn_stop = _ButtonProxy(self._can_press_stop, self._stop)
        self.btn_identify = _ButtonProxy(self._can_press_maintenance, self._identify_gravity)
        self.btn_seed = _ButtonProxy(self._can_press_maintenance, self._seed_gravity_from_urdf)
        self.btn_calib = _ButtonProxy(self._can_press_maintenance, self._auto_calib_gravity)
        self.btn_friction = _ButtonProxy(self._can_press_maintenance, self._identify_friction)
        self.btn_motor = _ButtonProxy(self._can_press_maintenance, self._read_motor_params)
        self.radio_sim = _RadioProxy(self, "sim")
        self.radio_real = _RadioProxy(self, "real")
        self.radio_moveit = _RadioProxy(self, "moveit")
        self.radio_zerog = _RadioProxy(self, "zerog")
        self.safety = _SafetyProxy(self)
        self.quest_profile = _ProfileProxy(self)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.setInterval(100)
        self._capture_socket = QUdpSocket(self)
        capture_port = int(os.environ.get("OPENARM_QUEST_CAPTURE_PORT", "47934"))
        capture_host = quest_udp_host()
        if capture_host == "0.0.0.0":
            capture_address = QHostAddress(QHostAddress.SpecialAddress.AnyIPv4)
        elif capture_host == "127.0.0.1":
            capture_address = QHostAddress(QHostAddress.SpecialAddress.LocalHost)
        else:
            capture_address = QHostAddress(capture_host)
        if not capture_address.isNull() and self._capture_socket.bind(capture_address, capture_port):
            self._capture_socket.readyRead.connect(self._capture_shortcut)
        else:
            self._set_record_hint(f"手柄采集快捷键不可用：{capture_host}:{capture_port} 无法绑定。")

        self._batch_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self._replay_bridge = ReplayBridge(self)
        self._replay_bridge.changed.connect(self._emit_ui)
        self._joint_angles = JointAngles(self)
        self._motors = MotorTable(self)
        self._motor_message = "尚未读取。查询 EEPROM，不使能、不动臂。"
        self._motor_report = ""
        self._refresh_usb()
        self._apply_mode()
        if self._page == "data":
            self._catalog.refresh()

    # --- properties -----------------------------------------------------

    @Property(str, notify=modeChanged)
    def mode(self) -> str:
        return self._mode_name

    @Property(str, notify=modeChanged)
    def modeLabel(self) -> str:
        return _MODE_LABELS.get(self._mode_name, self._mode_name)

    @Property(bool, notify=uiChanged)
    def running(self) -> bool:
        return self._running()

    @Property(str, notify=statusTextChanged)
    def statusText(self) -> str:
        return self._status_text

    @Property(str, notify=statusToneChanged)
    def statusTone(self) -> str:
        return self._status_tone

    @Property(str, notify=recordHintChanged)
    def recordHint(self) -> str:
        return self._record_hint

    @Property(str, notify=recordStateChanged)
    def recordState(self) -> str:
        return self._record_state

    @Property(str, notify=recordStateChanged)
    def recordLabel(self) -> str:
        return _RECORD_LABELS.get(self._record_state, "未就绪")

    @Property(str, notify=questProfileChanged)
    def questProfile(self) -> str:
        return self._quest_profile

    @questProfile.setter
    def questProfile(self, value: str) -> None:
        self.setQuestProfile(value)

    @Property(bool, notify=uiChanged)
    def questProfileEnabled(self) -> bool:
        return self._mode_name in ("sim", "real") and not self._running()

    @Property(bool, notify=safetyCheckedChanged)
    def safetyChecked(self) -> bool:
        return self._safety

    @safetyChecked.setter
    def safetyChecked(self, value: bool) -> None:
        self.setSafetyChecked(bool(value))

    @Property(str, notify=batchNameChanged)
    def batchName(self) -> str:
        return self._batch_name

    @batchName.setter
    def batchName(self, value: str) -> None:
        self._set_batch_name(value)

    @Property(bool, notify=uiChanged)
    def batchEditable(self) -> bool:
        return self._record_available() and not self._awaiting_label and not self._recording

    @Property(str, notify=canChanged)
    def leftCan(self) -> str:
        return self._left_can

    @leftCan.setter
    def leftCan(self, value: str) -> None:
        self._set_can("_left_can", value)

    @Property(str, notify=canChanged)
    def rightCan(self) -> str:
        return self._right_can

    @rightCan.setter
    def rightCan(self, value: str) -> None:
        self._set_can("_right_can", value)

    @Property(str, notify=canChanged)
    def bitrate(self) -> str:
        return self._bitrate

    @bitrate.setter
    def bitrate(self, value: str) -> None:
        self._set_can("_bitrate", value)

    @Property(str, notify=canChanged)
    def dbitrate(self) -> str:
        return self._dbitrate

    @dbitrate.setter
    def dbitrate(self, value: str) -> None:
        self._set_can("_dbitrate", value)

    @Property(bool, notify=uiChanged)
    def canFieldsEnabled(self) -> bool:
        return self._mode_name in ("real", "moveit", "zerog") and not self._running()

    @Property(bool, notify=uiChanged)
    def showConnection(self) -> bool:
        return self._mode_name in ("real", "moveit", "zerog")

    @Property(bool, notify=uiChanged)
    def showImpedance(self) -> bool:
        return self._mode_name == "moveit"

    @Property(bool, notify=uiChanged)
    def impedanceProfileEnabled(self) -> bool:
        return self._mode_name == "moveit" and not self._running()

    @Property(bool, notify=uiChanged)
    def impedanceLive(self) -> bool:
        return (
            self._mode_name == "moveit"
            and self._running()
            and self._moveit_real
            and not self._impedance_busy
        )

    @Property(str, notify=impedanceChanged)
    def impedanceProfile(self) -> str:
        return self._impedance_profile

    @impedanceProfile.setter
    def impedanceProfile(self, value: str) -> None:
        if value in IMPEDANCE_LABELS and value != "drag" and value != self._impedance_profile:
            self._impedance_profile = value
            self.impedanceChanged.emit()

    @Property(str, constant=True)
    def questUrl(self) -> str:
        return f"Quest：https://{local_ip()}:{HTTP_PORT}"

    @Property(list, notify=camerasChanged)
    def cameraOptions(self):
        return ["（不选）", *self._usb_by_label.keys()]

    @Property(str, notify=camerasChanged)
    def headCamera(self) -> str:
        return self._camera_choice["head"]

    @headCamera.setter
    def headCamera(self, value: str) -> None:
        self._set_camera("head", value)

    @Property(str, notify=camerasChanged)
    def wristLeftCamera(self) -> str:
        return self._camera_choice["wrist_left"]

    @wristLeftCamera.setter
    def wristLeftCamera(self, value: str) -> None:
        self._set_camera("wrist_left", value)

    @Property(str, notify=camerasChanged)
    def wristRightCamera(self) -> str:
        return self._camera_choice["wrist_right"]

    @wristRightCamera.setter
    def wristRightCamera(self, value: str) -> None:
        self._set_camera("wrist_right", value)

    @Property(str, notify=cameraStatusChanged)
    def cameraStatus(self) -> str:
        return self._camera_status

    @Property(str, notify=pageChanged)
    def page(self) -> str:
        return self._page

    @page.setter
    def page(self, value: str) -> None:
        self.setPage(value)

    @Property(bool, notify=uiChanged)
    def canStart(self) -> bool:
        return self._can_press_start()

    @Property(bool, notify=uiChanged)
    def canStop(self) -> bool:
        return self._can_press_stop()

    @Property(bool, notify=uiChanged)
    def canRecordStart(self) -> bool:
        return self._can_record_start()

    @Property(bool, notify=uiChanged)
    def canRecordStop(self) -> bool:
        return self._can_record_stop()

    @Property(bool, notify=uiChanged)
    def canLabel(self) -> bool:
        return self._can_label()

    @Property(bool, notify=uiChanged)
    def canRedo(self) -> bool:
        return self._can_press_redo()

    @Property(str, notify=uiChanged)
    def redoLabel(self) -> str:
        return self._redo_label()

    @Property(bool, notify=uiChanged)
    def canMaintain(self) -> bool:
        return self._can_press_maintenance()

    @Property(bool, notify=logExpandedChanged)
    def logExpanded(self) -> bool:
        return self._log_expanded

    @logExpanded.setter
    def logExpanded(self, value: bool) -> None:
        self.setLogExpanded(bool(value))

    @Property(int, notify=logHeightChanged)
    def logHeight(self) -> int:
        return self._log_height

    @Property(bool, notify=logDetachedChanged)
    def logDetached(self) -> bool:
        return self._log_detached

    @logDetached.setter
    def logDetached(self, value: bool) -> None:
        self.setLogDetached(bool(value))

    @Property(QObject, constant=True)
    def logLines(self):
        return self.log_panel.lines

    @Property(int, notify=logCountsChanged)
    def logWarnings(self) -> int:
        return self.log_panel.warnings

    @Property(int, notify=logCountsChanged)
    def logErrors(self) -> int:
        return self.log_panel.errors

    @Property(int, notify=logCountsChanged)
    def logUnread(self) -> int:
        return self.log_panel.unread

    @Property(str, notify=logCountsChanged)
    def logFilterError(self) -> str:
        return self.log_panel.filter_error.text()

    @Property(bool, notify=logCountsChanged)
    def logPaused(self) -> bool:
        return self.log_panel.pause.isChecked()

    @Property(QObject, constant=True)
    def catalog(self):
        return self._catalog

    @Property(QObject, constant=True)
    def replay(self):
        return self._replay_bridge

    @Property(QObject, constant=True)
    def motors(self):
        return self._motors

    @Property(QObject, constant=True)
    def jointAngles(self):
        return self._joint_angles

    @Property(str, notify=motorInfoChanged)
    def motorMessage(self) -> str:
        return self._motor_message

    @Property(str, notify=motorInfoChanged)
    def motorReport(self) -> str:
        return self._motor_report

    @Property(str, notify=motorInfoChanged)
    def motorSide(self) -> str:
        return self._motors.side

    @Slot(str)
    def setMotorSide(self, side: str) -> None:
        self._motors.set_side(side)
        self.motorInfoChanged.emit()

    # --- state helpers --------------------------------------------------

    def _mode(self) -> str:
        return self._mode_name

    def _force_mode(self, mode: str) -> None:
        if mode not in _MODES or mode == self._mode_name:
            return
        self._mode_name = mode
        self.modeChanged.emit()
        self._apply_mode()

    def _set_status(self, text: str, tone: str | None = None) -> None:
        if text != self._status_text:
            self._status_text = text
            self.statusTextChanged.emit()
        inferred = tone or self._tone_for(text)
        if inferred != self._status_tone:
            self._status_tone = inferred
            self.statusToneChanged.emit()

    def _tone_for(self, text: str) -> str:
        if any(word in text for word in ("失败", "严重", "FATAL", "超时")):
            return "error"
        if text.startswith("正在") or "启动中" in text:
            return "busy"
        if "运行中" in text:
            return "running"
        return "idle"

    def _set_record_hint(self, text: str) -> None:
        if text != self._record_hint:
            self._record_hint = text
            self.recordHintChanged.emit()

    def _set_batch_name(self, value: str) -> None:
        if value != self._batch_name:
            self._batch_name = value
            self.batchNameChanged.emit()

    def _set_can(self, attr: str, value: str) -> None:
        if getattr(self, attr) != value:
            setattr(self, attr, value)
            self.canChanged.emit()

    def _set_camera(self, name: str, value: str) -> None:
        if self._camera_choice.get(name) == value:
            return
        self._camera_choice[name] = value
        self.camerasChanged.emit()

    def _set_camera_status(self, text: str) -> None:
        if text != self._camera_status:
            self._camera_status = text
            self.cameraStatusChanged.emit()

    def _emit_ui(self) -> None:
        state = self._record_state_value()
        self.uiChanged.emit()
        if state != self._record_state:
            self._record_state = state
            self.recordStateChanged.emit()

    def _record_state_value(self) -> str:
        if self._recording:
            return "recording"
        if self._awaiting_label:
            return "pending"
        if self._record_available():
            return "ready"
        return "idle"

    def _replay_active(self) -> bool:
        bridge = getattr(self, "_replay_bridge", None)
        if bridge is None:
            return False
        return bridge.dialog._running() or bridge.dialog._stopping

    def _arm_held(self) -> bool:
        return self._running() or self._configuration_busy or self._replay_active()

    def _can_press_start(self) -> bool:
        return not self._arm_held()

    def _can_press_stop(self) -> bool:
        return self._running() and not self._stop_pending

    def _can_press_maintenance(self) -> bool:
        return not self._arm_held()

    def _can_record_start(self) -> bool:
        return self._record_available() and not self._awaiting_label and not self._recording

    def _can_record_stop(self) -> bool:
        return self._record_available() and self._recording

    def _can_label(self) -> bool:
        return self._record_available() and self._awaiting_label

    def _can_press_redo(self) -> bool:
        return self._record_available() and not self._recording and (self._can_redo or self._undo_open())

    def _redo_label(self) -> str:
        return "撤销丢弃" if self._undo_open() else "丢弃重录"

    def _hardware_mode(self, mode: str | None = None) -> bool:
        mode = mode or self._mode()
        if mode in ("real", "zerog"):
            return True
        if mode == "moveit":
            return self._safety
        return False

    def _running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _record_available(self) -> bool:
        mode = self._mode()
        if not self._running() or mode == "moveit":
            return False
        if mode == "zerog":
            return True
        if mode in ("sim", "real"):
            return self._quest_ready
        return False

    def _apply_mode(self) -> None:
        mode = self._mode()
        if not self._running():
            if mode == "sim" and self._safety:
                self._safety = False
                self.safetyCheckedChanged.emit()
            if mode == "moveit":
                self._set_status(
                    "MoveIt 实机（已勾选急停）" if self._safety else "MoveIt 仿真（未勾选急停）"
                )
            elif mode == "zerog":
                self._set_status("零重力：勾选急停后启动（含关节补偿，不加摩擦）")
            elif self._status_text.startswith(("MoveIt", "零重力")):
                self._set_status("未启动")
        self._apply_record_state()
        self._apply_impedance_state()

    def _apply_impedance_state(self) -> None:
        self._impedance_busy = False
        self.impedanceChanged.emit()
        self._emit_ui()

    def _apply_record_state(self) -> None:
        self._emit_ui()

    def _append_log(self, text: str, source: str = "控制台") -> None:
        self.log_panel.enqueue(text, source)

    def _notice(self, text: str) -> None:
        self._set_status(text[:120])
        self._append_log(text if text.endswith("\n") else text + "\n")

    def _log_fatal(self, text: str) -> None:
        self._set_status("严重错误：" + text[:100], "error")
        self.setLogExpanded(True)

    def _undo_open(self) -> bool:
        return time.monotonic() < self._undo_until

    def _expire_undo(self) -> None:
        if self._undo_until and not self._undo_open():
            self._undo_until = 0.0
            self._apply_record_state()
            if self._record_hint == DISCARD_HINT:
                self._set_record_hint(UNDO_EXPIRED_HINT)

    def _sync_catalog(self) -> None:
        if self._catalog._loaded:
            self._catalog.refresh()

    def _save_settings(self) -> None:
        self.log_panel.set_expanded(self._log_expanded)
        self._settings.setValue("page", self._page)
        self._settings.setValue("log/height", self._log_height)
        self._settings.setValue("log/detached", self._log_detached)
        for source in list(self.log_panel.partials):
            self.log_panel.enqueue("", source, final=True)
        self.log_panel.flush()
        self.log_panel.save_settings()

    def close(self) -> bool:
        self._poll_timer.stop()
        self._save_settings()
        if self._capture_socket is not None:
            self._capture_socket.close()
        return True

    # --- slots used by QML ----------------------------------------------

    @Slot(str)
    def setMode(self, mode: str) -> None:
        if self._running():
            return
        self._force_mode(mode)

    @Slot(str)
    def setPage(self, page: str) -> None:
        if page not in _PAGES or page == self._page:
            return
        self._page = page
        self._settings.setValue("page", page)
        self.pageChanged.emit()
        if page == "data":
            self._catalog.refresh()
        elif page == "teach" and not self._running():
            self._force_mode("zerog")

    @Slot(str)
    def setQuestProfile(self, value: str) -> None:
        if self._running() or value not in {item for _title, item in _QUEST_PROFILES}:
            return
        if value != self._quest_profile:
            self._quest_profile = value
            self.questProfileChanged.emit()

    @Slot(bool)
    def setSafetyChecked(self, checked: bool) -> None:
        checked = bool(checked)
        if checked == self._safety:
            return
        self._safety = checked
        self.safetyCheckedChanged.emit()
        self._apply_mode()

    @Slot()
    def start(self) -> None:
        self._start()

    @Slot()
    def goTeachHome(self) -> None:
        if self._mode() != "zerog" or not self._running():
            QMessageBox.warning(None, "拖动示教", "请先启动拖动示教，再前往起手姿势。")
            return
        if self._recording:
            QMessageBox.warning(None, "拖动示教", "请先结束录制，再前往起手姿势。")
            return
        try:
            send_record("home")
        except OSError as error:
            QMessageBox.warning(None, "拖动示教", f"无法发送前往起手姿势：{error}")
            return
        self._set_status("正在前往起手姿势…")
        self._set_record_hint("正在沿起手轨迹移动。到达后保持左右镜像，点开始录制后才松开，然后可以拖动。")

    @Slot()
    def startDragTeach(self) -> None:
        if self._running() and self._mode() != "zerog":
            QMessageBox.warning(None, "拖动示教", "请先停止当前模式。拖动示教需要独占机械臂。")
            return
        if not self._running():
            self._force_mode("zerog")
            self._start()
        self.setPage("teach")

    @Slot()
    def stop(self) -> None:
        self._stop()

    @Slot(str)
    def record(self, kind: str) -> None:
        self._record(kind)

    @Slot()
    def redo(self) -> None:
        self._on_redo_button()

    @Slot()
    def seedGravity(self) -> None:
        self._seed_gravity_from_urdf()

    @Slot()
    def autoCalib(self) -> None:
        self._auto_calib_gravity()

    @Slot()
    def identifyGravity(self) -> None:
        self._identify_gravity()

    @Slot()
    def identifyFriction(self) -> None:
        self._identify_friction()

    @Slot()
    def readMotors(self) -> None:
        self._read_motor_params()

    @Slot(str)
    def setImpedance(self, name: str) -> None:
        self._set_impedance_profile(name)

    @Slot()
    def softStop(self) -> None:
        self._soft_stop_moveit()

    @Slot()
    def openReplay(self) -> None:
        self._open_replay()

    @Slot()
    def refreshCameras(self) -> None:
        self._refresh_usb()

    @Slot()
    def openCameras(self) -> None:
        self._open_cameras()

    @Slot(bool)
    def setLogExpanded(self, expanded: bool) -> None:
        expanded = bool(expanded)
        if expanded == self._log_expanded:
            return
        self._log_expanded = expanded
        self.log_panel.set_expanded(expanded)
        self.logExpandedChanged.emit()

    @Slot(int)
    def setLogHeight(self, height: int) -> None:
        height = max(120, int(height))
        if self._viewport_height:
            height = min(height, max(120, int(self._viewport_height * 0.6)))
        if height != self._log_height:
            self._log_height = height
            self.logHeightChanged.emit()

    @Slot(int)
    def setViewportHeight(self, height: int) -> None:
        self._viewport_height = max(0, int(height))

    @Slot(bool)
    def setLogDetached(self, detached: bool) -> None:
        detached = bool(detached)
        if detached == self._log_detached:
            return
        self._log_detached = detached
        self.logDetachedChanged.emit()
        if detached:
            self.setLogExpanded(True)

    @Slot(str)
    def setLogFilter(self, text: str) -> None:
        self.log_panel.filter.setText(text)

    @Slot(str)
    def setLogSource(self, text: str) -> None:
        self.log_panel.source.setCurrentText(text)

    @Slot(bool)
    def setLogRegex(self, enabled: bool) -> None:
        self.log_panel.regex.setChecked(enabled)

    @Slot(bool)
    def setLogPaused(self, paused: bool) -> None:
        self.log_panel.pause.setChecked(paused)

    @Slot()
    def clearLog(self) -> None:
        self.log_panel.clear()

    @Slot()
    def followLog(self) -> None:
        self.log_panel.follow_latest()
        self.logCountsChanged.emit()

    @Slot()
    def nextLogError(self) -> None:
        self.setLogExpanded(True)
        self.log_panel.next_error()

    @Slot(str)
    def setLogFind(self, text: str) -> None:
        self.log_panel.find.setText(text)

    @Slot()
    def findNextLog(self) -> None:
        self.log_panel.find_next()

    @Slot(str, bool)
    def setLogLevelEnabled(self, level: str, enabled: bool) -> None:
        action = self.log_panel.levels.get(level)
        if action is not None:
            action.setChecked(enabled)

    @Slot(str, result=bool)
    def logLevelEnabled(self, level: str) -> bool:
        action = self.log_panel.levels.get(level)
        return bool(action is not None and action.isChecked())

    @Slot(result=bool)
    def prepareClose(self) -> bool:
        if self._running():
            if (
                QMessageBox.question(None, "退出", "遥操作还在运行。停止并退出？")
                != QMessageBox.StandardButton.Yes
            ):
                return False
            self._stop()
            QTimer.singleShot(300, self._wait_close)
            return False
        if self._replay_active():
            if (
                QMessageBox.question(None, "退出", "动作回放还在运行。停止并退出？")
                != QMessageBox.StandardButton.Yes
            ):
                return False
            self._replay_bridge.stopPlayback()
            QTimer.singleShot(300, self._wait_replay_close)
            return False
        self._save_settings()
        return True

    def _wait_replay_close(self) -> None:
        if self._replay_active():
            QTimer.singleShot(300, self._wait_replay_close)
            return
        self._save_settings()
        self.closeRequested.emit()

    def _wait_close(self) -> None:
        if self._running():
            QTimer.singleShot(300, self._wait_close)
            return
        self._save_settings()
        self.closeRequested.emit()

    # --- capture --------------------------------------------------------

    def _capture_shortcut(self) -> None:
        while self._capture_socket.hasPendingDatagrams():
            datagram = self._capture_socket.receiveDatagram()
            ok = False
            message = "当前不能执行此采集操作，请检查录制状态。"
            try:
                action = json.loads(bytes(datagram.data())).get("action")
                self._expire_undo()
                if self._capture_blocked_by_startup():
                    message = QUEST_NOT_READY_MESSAGE
                else:
                    ok, message = self._dispatch_capture(action, message)
            except (ValueError, AttributeError, TypeError):
                message = "无效的采集快捷键请求。"
            reply = json.dumps({"ok": ok, "message": message}, ensure_ascii=False).encode()
            self._capture_socket.writeDatagram(reply, datagram.senderAddress(), datagram.senderPort())

    def _capture_blocked_by_startup(self) -> bool:
        if self._mode() not in ("sim", "real") or self._quest_ready:
            return False
        return self._running() or self._status_text in ("正在启动…", "启动中…")

    def _dispatch_capture(self, action: str | None, message: str) -> tuple[bool, str]:
        button = {
            "left-short": self.rec_stop if self._recording else self.rec_start,
            "left-long": self.rec_redo,
            "right-short": self.rec_ok,
            "right-long": self.rec_bad,
        }.get(action)
        allowed = (
            self._mode() in ("sim", "real")
            and self._record_available()
            and button is not None
            and button.isEnabled()
            and not (action == "left-long" and self._recording)
        )
        if not allowed:
            return False, message
        if action == "left-long":
            kind = "undo" if self._undo_open() else "redo"
            return self._record(kind, quiet=True), self._record_hint
        if action == "left-short" and not self._recording and not self._batch_name.strip():
            return False, "请先在控制台填写批次名称。"
        kind = {
            "left-short": "stop" if self._recording else "start",
            "right-short": "success",
            "right-long": "fail",
        }[action]
        return self._record(kind, quiet=True), self._record_hint

    # --- cameras --------------------------------------------------------

    def _refresh_usb(self) -> None:
        cameras = list_usb_cameras()
        self._usb_by_label = {}
        for cam in cameras:
            label = cam.label()
            if label in self._usb_by_label:
                label = f"{label} · {cam.port}"
            self._usb_by_label[label] = cam
        values = ["（不选）", *self._usb_by_label]
        saved = {spec.name: spec for spec in load_specs(default_config())}
        for name in _CAMERA_KEYS:
            current = self._camera_choice[name]
            pick = "（不选）"
            spec = saved.get(name)
            if spec is not None:
                for label, cam in self._usb_by_label.items():
                    if spec.usb and cam.port == spec.usb:
                        pick = label
                        break
                    if spec.serial and cam.serial and spec.serial.upper() == cam.serial.upper():
                        pick = label
                        break
            if current in values:
                pick = current
            self._camera_choice[name] = pick
        self.camerasChanged.emit()
        if cameras:
            self._set_camera_status(f"读到 {len(cameras)} 个 Orbbec USB 口。选好后再开窗口。")
        else:
            self._set_camera_status("没有读到 Orbbec 相机。确认 Gemini 335 已插入后再刷新。")

    def _save_camera_choice(self) -> bool:
        if not self._usb_by_label:
            return True
        chosen = {}
        used: set[str] = set()
        for name in _CAMERA_KEYS:
            cam = self._usb_by_label.get(self._camera_choice[name])
            if cam is not None:
                if cam.port in used:
                    QMessageBox.warning(None, "相机", "头部、左腕、右腕不能选同一个 USB 口。")
                    return False
                used.add(cam.port)
            chosen[name] = cam
        save_specs(default_config(), chosen)
        return True

    def _open_cameras(self) -> None:
        if not self._save_camera_choice():
            return
        if camera_view_open():
            QMessageBox.information(
                None,
                "相机",
                "相机采集已运行（电脑预览或后台采集）。如需更换 USB 口或打开电脑预览窗口，请先停止遥操并关闭已有相机窗口，再重新打开。",
            )
            return
        log_path = Path(tempfile.gettempdir()) / "openarm_camera_view.log"
        handle = log_path.open("ab", buffering=0)
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        subprocess.Popen(
            [sys.executable, str(ROOT / "script" / "lerobot_cameras.py"), "view"],
            cwd=ROOT,
            env=environment,
            start_new_session=True,
            stdout=handle,
            stderr=subprocess.STDOUT,
        )
        handle.close()
        if not self._running():
            self._set_status("三个相机窗口已打开")

    def _ask_sudo(self) -> str | None:
        if sudo_is_cached():
            return ""
        text, ok = QInputDialog.getText(
            None,
            "配置真机 CAN",
            "请输入当前用户的 sudo 密码。\n密码只传给这一次 CAN 配置，不会保存。",
            QLineEdit.EchoMode.Password,
        )
        if not ok:
            return None
        if not text:
            QMessageBox.warning(None, "配置真机 CAN", "sudo 密码不能为空。")
            return None
        return text

    # --- launch / stop --------------------------------------------------

    def _start(self) -> None:
        if self._arm_held():
            return
        mode = self._mode()
        if mode in ("sim", "real") and any(
            self._usb_by_label.get(self._camera_choice[name]) is not None for name in _CAMERA_KEYS
        ):
            if not self._save_camera_choice():
                return
        real = self._hardware_mode(mode)
        left = self._left_can.strip()
        right = self._right_can.strip()
        bitrate = self._bitrate.strip()
        dbitrate = self._dbitrate.strip()
        if real:
            title = {"moveit": "MoveIt 实机", "zerog": "零重力", "real": "真机遥操"}.get(mode, "真机")
            if not self._safety:
                QMessageBox.warning(None, title, "启动前请先勾选安全确认。")
                return
            if not left or not right or left == right:
                QMessageBox.warning(None, title, "左右臂需要两个不同的 CAN 口。")
                return
            if not bitrate.isdigit() or not dbitrate.isdigit():
                QMessageBox.warning(None, title, "波特率必须是整数。")
                return
        password = None
        if real:
            password = self._ask_sudo()
            if password is None:
                return
        if mode == "moveit":
            self._set_status("正在启动 MoveIt 实机…" if real else "正在启动 MoveIt 仿真…")
        elif mode == "zerog":
            self._set_status("正在启动零重力…")
        else:
            self._set_status("正在启动…")
        self._configuration_busy = True
        self._quest_profile_requested = self._quest_profile
        self._moveit_profile_requested = self._impedance_profile
        self._emit_ui()
        self._joint_angles.clear()
        threading.Thread(
            target=self._start_worker,
            args=(mode, left, right, bitrate, dbitrate, password),
            daemon=True,
        ).start()

    def _start_worker(
        self,
        mode: str,
        left: str,
        right: str,
        bitrate: str,
        dbitrate: str,
        password: str | None,
    ) -> None:
        try:
            if self._hardware_mode(mode):
                configure_can(left, right, bitrate, dbitrate, password)
            self._spawn(mode, left, right)
        except Exception as error:  # noqa: BLE001
            self.bridge.finished_fail.emit(str(error))
            return
        self.bridge.finished_ok.emit()

    def _spawn(self, mode: str, left: str, right: str) -> None:
        self._quest_ready = False
        self._undo_until = 0.0
        environment = os.environ.copy()
        environment["OPENARM_RECORD_PANEL"] = "0"
        environment.pop("OPENARM_JOINT_ANGLE_PORT", None)
        if self._joint_angles.available:
            environment["OPENARM_JOINT_ANGLE_PORT"] = str(self._joint_angles.socket.localPort())
        environment["OPENARM_LEFT_FOLLOWER_CAN"] = left
        environment["OPENARM_RIGHT_FOLLOWER_CAN"] = right
        environment.setdefault("OPENARM_ROOT", str(ROOT))
        environment.setdefault("OPENARM_GRAVITY_YAML", str(ROOT / "config" / "gravity_comp.yaml"))
        self._log_reader_reset()
        self._log_ready_tail = ""
        self.log_handle = self.log_path.open("wb", buffering=0)
        if mode == "moveit":
            use_real = self._hardware_mode(mode)
            environment["OPENARM_MOVEIT_REAL"] = "true" if use_real else "false"
            environment["OPENARM_MOVEIT_PROFILE"] = str(self._moveit_profile_requested or "P0")
            environment["OPENARM_MOVEIT_COMPLIANT"] = "true"
            command = [
                "bash",
                str(ROOT / "scripts" / "run_moveit.sh"),
                "--real" if use_real else "--sim",
            ]
        elif mode == "zerog":
            command = ["bash", str(ROOT / "scripts" / "run_zero_g.sh")]
        else:
            command = ["bash", str(RUN), "--real" if mode == "real" else "--sim"]
            environment["VR_TELEOP_MODE"] = "unitree_style"
        self.proc = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def _log_reader_reset(self) -> None:
        self._log_reader.reset()

    def _started(self) -> None:
        self._configuration_busy = False
        mode = self._mode()
        if mode == "moveit":
            kind = "MoveIt 实机" if self._safety else "MoveIt 仿真"
        elif mode == "zerog":
            kind = "零重力"
        else:
            kind = {"real": "真机", "sim": "仿真"}.get(mode, "仿真")
        if mode in ("sim", "real"):
            self._set_status("启动中…")
        else:
            self._set_status(f"{kind}运行中")
        if mode == "zerog":
            self._set_record_hint("拖动示教已托住双臂。先拖到起点，再点开始录制。")
        elif mode == "moveit" and self._safety:
            self._moveit_real = True
            self._set_record_hint("MoveIt 实机：可用软硬或软急停。停止并释放会失能下落。")
        elif mode == "moveit":
            self._set_record_hint("MoveIt 仿真只在 RViz 里动模型，不录遥操数据。")
        elif mode in ("sim", "real"):
            self._set_record_hint(QUEST_NOT_READY_MESSAGE)
        else:
            self._set_record_hint("可以开始录。停下来之后选择成功、失败或重录。")
        self._awaiting_label = False
        self._recording = False
        self._can_redo = False
        self._undo_until = 0.0
        self._apply_record_state()
        self._poll_timer.start()
        self._sync_catalog()
        if mode in ("sim", "real"):
            self._refresh_log()

    def _quest_ready_hint(self) -> str:
        return "可以开始录。停下来之后选择成功、失败或重录。"

    def _fail(self, text: str) -> None:
        self._configuration_busy = False
        self._quest_ready = False
        self._undo_until = 0.0
        self._moveit_real = False
        self._set_status("启动失败")
        self._apply_mode()
        QMessageBox.critical(None, "启动失败", text)

    def _stop(self) -> None:
        proc = self.proc
        if proc is None or proc.poll() is not None:
            return
        self._set_status("正在释放…")
        self._stop_pending = True
        self._emit_ui()

        def work() -> None:
            stuck = False
            try:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        stuck = True
            except ProcessLookupError:
                stuck = False
            except OSError:
                stuck = proc.poll() is None
            if stuck:
                self.bridge.stop_stuck.emit("释放超时，进程仍在运行。可再次停止。")
            else:
                self.bridge.idle.emit("已停止")

        threading.Thread(target=work, daemon=True).start()

    def _stop_stuck(self, text: str) -> None:
        proc = self.proc
        if proc is None:
            return
        if proc.poll() is not None:
            self._idle("已停止")
            return
        self._stop_pending = False
        self._set_status(text)
        self._apply_record_state()

    def _idle(self, text: str) -> None:
        self._refresh_log(final=True)
        while self.log_path.is_file() and self._log_reader.offset < self.log_path.stat().st_size:
            self._refresh_log(final=True)
        self.log_panel.enqueue("", "控制器", final=True)
        self.log_panel.flush()
        self.proc = None
        self._close_log()
        self._awaiting_label = False
        self._recording = False
        self._can_redo = False
        self._quest_ready = False
        self._undo_until = 0.0
        self._moveit_real = False
        self._stop_pending = False
        self._set_status(text)
        self._set_record_hint("启动之后才能录。停下来再标成功、失败，或重录。")
        self._poll_timer.stop()
        self._apply_mode()
        self._apply_record_state()
        self._sync_catalog()

    def _poll(self) -> None:
        self._expire_undo()
        self._refresh_log()
        proc = self.proc
        if proc is None:
            self._poll_timer.stop()
            return
        code = proc.poll()
        if code is not None:
            if code:
                self._append_log(f"[FATAL] 控制器异常退出，返回码 {code}\n")
            self._idle(f"控制器已退出，返回码 {code}")

    def _refresh_log(self, *, final: bool = False) -> None:
        if not self.log_path.is_file():
            return
        generation = self._log_reader.generation
        text = self._log_reader.read(self.log_path, final=final)
        if generation != self._log_reader.generation:
            self._log_ready_tail = ""
            self.log_panel.partials.pop("控制器", None)
        ready_text = self._log_ready_tail + text
        self._log_ready_tail = ready_text[-4096:]
        if self._mode() in ("sim", "real") and not self._quest_ready and self._running():
            if _QUEST_RUNNING_RE.search(ready_text):
                self._quest_ready = True
                kind = "真机" if self._mode() == "real" else "仿真"
                self._set_status(f"{kind}运行中")
                self._set_record_hint(self._quest_ready_hint())
                self._apply_record_state()
                self._sync_catalog()
        if text:
            self.log_panel.enqueue(text, "控制器")

    def _close_log(self) -> None:
        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None

    # --- gravity / motors -----------------------------------------------

    def _identify_gravity(self) -> None:
        title = "惯量标定"
        if (
            self._running()
            or self._configuration_busy
            or not self.btn_start.isEnabled()
            or self._calib_proc is not None
        ):
            QMessageBox.warning(None, title, "请先停止控制器，并等待当前操作完成。")
            return
        log_path = latest_two_sided_log()
        if log_path is None:
            QMessageBox.warning(
                None,
                title,
                "没有找到两侧接近的标定日志。\n请先运行一次「自动微调」采集数据，再做惯量标定。",
            )
            return
        self._identify_log = log_path
        self._start_identification(log_path, write=False)

    def _start_identification(self, log_path: Path, write: bool) -> None:
        yaml_path = resolve_gravity_yaml()
        self._configuration_busy = True
        self._emit_ui()
        self._set_status("惯量标定写入中…" if write else "惯量标定计算中…")
        self._append_log(f"\n--- 惯量标定{'写入' if write else '预览'}：{log_path.name} ---\n")

        def work() -> None:
            try:
                ok, output = run_identification(log_path, yaml_path, write)
            except Exception as error:  # noqa: BLE001
                ok, output = False, str(error)
            self.bridge.identify_done.emit(write, ok, output)

        threading.Thread(target=work, daemon=True).start()

    def _identify_done(self, write: bool, ok: bool, output: str) -> None:
        self._configuration_busy = False
        self._emit_ui()
        title = "惯量标定"
        self._append_log(output + "\n", "标定")
        if not ok:
            self._set_status(f"{title}失败")
            shown = output if len(output) < 4000 else ("…\n" + output[-4000:])
            QMessageBox.critical(None, title, shown)
            return
        if write:
            self._set_status("惯量标定已写入；下次启动生效")
            QMessageBox.information(
                None,
                title,
                "已写入辨识结果，下次启动生效。\n"
                "下一步：零重力下逐姿态验证左右是否一致，再用「自动微调」小幅精修。",
            )
            return
        self._set_status("惯量标定预览完成，未写入")
        if QMessageBox.question(
            None,
            title,
            f"日志：{self._identify_log.name}\n留出姿态误差 RMS (Nm, j1..j7)：\n\n"
            f"{identify_summary(output)}\n\n"
            "写入后将用辨识的连杆质量/质心替换当前修正参数（远端增益、倍率、偏置归一），"
            "直接覆盖当前配置，下次启动生效。\n"
            "若出现「已夹限」，建议先检查夹爪/相机安装状态再决定。\n是否写入？",
        ) == QMessageBox.StandardButton.Yes:
            self._start_identification(self._identify_log, write=True)

    def _seed_gravity_from_urdf(self) -> None:
        title = "使用 URDF 初值"
        if (
            self._running()
            or self._configuration_busy
            or not self.btn_start.isEnabled()
            or self._calib_proc is not None
        ):
            QMessageBox.warning(None, title, "请先停止控制器，并等待当前操作完成。")
            return
        yaml_path = resolve_gravity_yaml()
        urdf_path = Path(os.environ.get("OPENARM_TELEOP_URDF", str(ROOT / "assets" / "v1.urdf"))).expanduser()
        if not urdf_path.is_absolute():
            urdf_path = ROOT / urdf_path
        if QMessageBox.question(
            None,
            title,
            "将当前重力补偿替换为 URDF 模型初值？\n\n"
            "质量/质心、逐关节倍率和偏置将恢复为模型基准；前馈增益设为 0.95。\n"
            "夹爪/相机安装状态、负载及摩擦参数会保留；当前调好的参数将被直接覆盖。\n"
            "此操作只写配置，不会驱动机械臂；下次启动生效。\n"
            "下一步：运行「自动微调」采数，再做「惯量标定」。\n\n"
            f"URDF：{urdf_path}\n配置：{yaml_path}",
        ) != QMessageBox.StandardButton.Yes:
            return
        try:
            output = install_urdf_gravity_seed(yaml_path, urdf_path)
        except Exception as error:  # noqa: BLE001
            self._set_status("URDF 初值设置失败")
            QMessageBox.critical(None, title, str(error))
            return
        self._append_log(f"\n--- URDF 初值 ---\n{output}\n", "标定")
        self._set_status("URDF 初值已写入；下次启动生效")
        QMessageBox.information(
            None,
            title,
            "已写入 URDF 初值，下次启动生效。\n下一步：运行「自动微调」采数，再做「惯量标定」。",
        )

    def _auto_calib_gravity(self) -> None:
        title = "自动微调"
        if self._arm_held():
            QMessageBox.warning(None, title, "请先停止当前模式，标定需要独占 CAN。")
            return
        if self._calib_proc is not None and self._calib_proc.poll() is None:
            QMessageBox.warning(None, title, "标定正在进行中。")
            return
        if not self._safety:
            QMessageBox.warning(None, title, "标定前请勾选安全确认。")
            return
        left = self._left_can.strip()
        right = self._right_can.strip()
        bitrate = self._bitrate.strip()
        dbitrate = self._dbitrate.strip()
        if not left or not right or left == right:
            QMessageBox.warning(None, title, "左右臂需要两个不同的 CAN 口。")
            return
        if not bitrate.isdigit() or not dbitrate.isdigit():
            QMessageBox.warning(None, title, "波特率必须是整数。")
            return
        yaml_path = resolve_gravity_yaml()
        if not yaml_path.is_file():
            QMessageBox.warning(None, title, f"找不到当前重力配置：{yaml_path}")
            return
        if (
            QMessageBox.question(
                None,
                title,
                "在当前配置上自动微调（时长随停稳情况变化，机械臂会运动）：\n"
                "1. 腕→肘→肩各姿态分别从 +δ、−δ 两侧接近，停稳后低刚度软持采数（移动时不记录）\n"
                "   包含肩部垂直附近 4 个姿态；双臂位置/保持力矩稳定时缩短等待和采数\n"
                "2. 软持结束先原地变硬，再平滑移到下一起点，不会猛拉回位\n"
                "3. 按零重力同一 KDL 模型小幅调整关节倍率/偏置；质量/质心、负载和摩擦不变\n"
                "4. 用未参与拟合的姿态验证，改善才直接写回当前配置\n"
                "采数日志可随后用于「惯量标定」。\n\n"
                f"目标：{yaml_path}\n\n"
                "工作区清空，手勿碰臂。继续？",
            )
            != QMessageBox.StandardButton.Yes
        ):
            return
        password = self._ask_sudo()
        if password is None:
            return
        self._configuration_busy = True
        self._emit_ui()
        self._set_status("正在自动微调…（日志区有实时输出）")
        self._append_log("\n--- 自动微调开始 ---\n", "标定")
        threading.Thread(
            target=self._auto_calib_worker,
            args=(left, right, bitrate, dbitrate, password, yaml_path),
            daemon=True,
        ).start()

    def _auto_calib_worker(
        self,
        left: str,
        right: str,
        bitrate: str,
        dbitrate: str,
        password: str | None,
        yaml_path: Path,
    ) -> None:
        script = ROOT / "scripts" / "calibrate_gravity.py"
        chunks: list[str] = []
        try:
            configure_can(left, right, bitrate, dbitrate, password)
            env = os.environ.copy()
            env["OPENARM_LEFT_FOLLOWER_CAN"] = left
            env["OPENARM_RIGHT_FOLLOWER_CAN"] = right
            env["OPENARM_ROOT"] = str(ROOT)
            env["OPENARM_GRAVITY_YAML"] = str(yaml_path)
            env["PYTHONUNBUFFERED"] = "1"
            self._calib_proc = subprocess.Popen(
                [sys.executable, "-u", str(script), "--trajectory", "waypoints", "--write", str(yaml_path)],
                cwd=ROOT,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert self._calib_proc.stdout is not None
            for line in self._calib_proc.stdout:
                chunks.append(line)
                self.bridge.calib_log.emit(line)
            code = self._calib_proc.wait()
            ok = code == 0
            output = "".join(chunks).strip()
            if not ok and not output:
                output = f"标定进程退出码 {code}"
        except Exception as error:  # noqa: BLE001
            output = str(error)
            ok = False
        finally:
            self._calib_proc = None
        self.bridge.calib_done.emit(ok, output)

    def _calib_log_line(self, line: str) -> None:
        self._append_log(line, "标定")
        if "backend=" in line or "自动标定完成" in line or "LOG_PATH=" in line:
            self._set_status(line.strip()[:120])

    def _auto_calib_done(self, ok: bool, output: str) -> None:
        self.log_panel.enqueue("", "标定", final=True)
        self.log_panel.flush()
        self._configuration_busy = False
        self._emit_ui()
        title = "自动微调"
        if not ok:
            self._set_status(f"{title}失败")
            shown = output if len(output) < 4000 else ("…\n" + output[-4000:])
            QMessageBox.critical(None, title, shown or "失败")
            return
        self._set_status(f"{title}完成")
        QMessageBox.information(
            None,
            title,
            "已完成关节倍率/偏置微调与独立姿态验证；未改善的关节保留原值。\n"
            "从 URDF 初值开始时：下一步点「惯量标定」，用本次日志辨识质量/质心。\n"
            "已做过惯量标定时：启动「零重力」检查手感。",
        )

    def _identify_friction(self, gripper: bool = False) -> None:
        title = "夹爪摩擦辨识" if gripper else "摩擦辨识"
        if self._arm_held():
            QMessageBox.warning(None, title, "请先停止当前模式，摩擦辨识需要独占 CAN。")
            return
        if self._calib_proc is not None and self._calib_proc.poll() is None:
            QMessageBox.warning(None, title, "标定正在进行中。")
            return
        if not self._safety:
            QMessageBox.warning(None, title, f"{title}前请勾选安全确认。")
            return
        left = self._left_can.strip()
        right = self._right_can.strip()
        bitrate = self._bitrate.strip()
        dbitrate = self._dbitrate.strip()
        if not left or not right or left == right:
            QMessageBox.warning(None, title, "左右臂需要两个不同的 CAN 口。")
            return
        if not bitrate.isdigit() or not dbitrate.isdigit():
            QMessageBox.warning(None, title, "波特率必须是整数。")
            return
        yaml_path = resolve_gravity_yaml()
        if not yaml_path.is_file():
            QMessageBox.warning(None, title, f"找不到当前重力配置：{yaml_path}")
            return
        if gripper:
            prompt = (
                "夹爪会开合，大约 1 分钟，手臂保持不动：\n"
                "1. 先慢慢探到夹爪自己的开合限位并退回，再以 0.10、0.25、0.50 rad/s 正反向扫描\n"
                "   夹爪里不要放物体\n"
                "2. 顶住不动或跟踪误差过大立即停止\n"
                "3. 左右按库仑平台拟合，写入通过侧里较小的值，打 85 折\n"
                "只改 gripper_friction_fc、gripper_friction_fv 和 gripper_friction_k。\n\n"
                f"目标：{yaml_path}\n\n"
                "手勿碰夹爪。继续？"
            )
        else:
            prompt = (
                "双臂会逐个关节来回转动，大约 8–10 分钟：\n"
                "1. J1→J7 依次以 0.05、0.15、0.30 rad/s 正反向各走一遍，其余关节按住\n"
                "2. 位置硬跟踪，只加重力前馈；跟踪误差超过 0.15 rad 立即停止\n"
                "3. 同一位置正反向力矩相减消去重力，拟合库仑 Fc 与黏性 Fv\n"
                "4. 先预览，确认后才写入；写入值按拟合结果打 85 折，换向陡度写成 8\n"
                "   拟合超过上限的关节保留原值，避免零速来回推\n"
                "只改 left/right_friction_fc、friction_fv 和通过关节的 friction_k。\n\n"
                f"目标：{yaml_path}\n\n"
                "工作区清空，手勿碰臂。继续？"
            )
        if QMessageBox.question(None, title, prompt) != QMessageBox.StandardButton.Yes:
            return
        password = self._ask_sudo()
        if password is None:
            return
        self._configuration_busy = True
        self._emit_ui()
        self._set_status(f"正在{title}采数…（日志区有实时输出）")
        self._append_log(f"\n--- {title}开始 ---\n", "标定")
        threading.Thread(
            target=self._friction_worker,
            args=(left, right, bitrate, dbitrate, password, yaml_path, gripper),
            daemon=True,
        ).start()

    def _friction_worker(
        self,
        left: str,
        right: str,
        bitrate: str,
        dbitrate: str,
        password: str | None,
        yaml_path: Path,
        gripper: bool = False,
    ) -> None:
        chunks: list[str] = []
        try:
            configure_can(left, right, bitrate, dbitrate, password)
            env = os.environ.copy()
            env["OPENARM_LEFT_FOLLOWER_CAN"] = left
            env["OPENARM_RIGHT_FOLLOWER_CAN"] = right
            env["OPENARM_ROOT"] = str(ROOT)
            env["OPENARM_GRAVITY_YAML"] = str(yaml_path)
            env["PYTHONUNBUFFERED"] = "1"
            self._calib_proc = subprocess.Popen(
                friction_collect_command(yaml_path, gripper=gripper),
                cwd=ROOT,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert self._calib_proc.stdout is not None
            for line in self._calib_proc.stdout:
                chunks.append(line)
                self.bridge.calib_log.emit(line)
            code = self._calib_proc.wait()
            ok = code == 0
            output = "".join(chunks).strip()
            if not ok and not output:
                output = f"摩擦辨识进程退出码 {code}"
        except Exception as error:  # noqa: BLE001
            output = str(error)
            ok = False
        finally:
            self._calib_proc = None
        self.bridge.friction_done.emit(False, ok, output)

    def _friction_done(self, write: bool, ok: bool, output: str) -> None:
        self.log_panel.enqueue("", "标定", final=True)
        self.log_panel.flush()
        self._configuration_busy = False
        self._emit_ui()
        title = "夹爪摩擦辨识" if "夹爪" in output else "摩擦辨识"
        if write:
            self._append_log(output + "\n", "标定")
        if not ok:
            self._set_status(f"{title}失败")
            shown = output if len(output) < 4000 else ("…\n" + output[-4000:])
            QMessageBox.critical(None, title, shown or "失败")
            return
        if write:
            self._set_status("摩擦参数已写入；重力补偿不下发")
            QMessageBox.information(
                None,
                title,
                "已写入摩擦参数，保存在配置里。\n当前重力补偿不加这些摩擦力矩。",
            )
            return
        self._friction_log = friction_log_path(output)
        details = friction_summary(output)
        if self._friction_log is None or "→" not in details:
            self._set_status("摩擦辨识完成，没有可写入的关节")
            QMessageBox.warning(None, title, (details or output[-2000:]) + "\n\n配置未改动。")
            return
        self._set_status("摩擦辨识预览完成，未写入")
        if QMessageBox.question(
            None,
            title,
            f"日志：{self._friction_log.name}\n\n{details}\n\n"
            "标「保留」的关节不改。写入后保存在配置里，重力补偿不会加到前馈上。是否写入？",
        ) != QMessageBox.StandardButton.Yes:
            return
        log_path = self._friction_log
        yaml_path = resolve_gravity_yaml()
        self._configuration_busy = True
        self._emit_ui()
        self._set_status("摩擦参数写入中…")

        def work() -> None:
            try:
                done, text = run_friction_write(log_path, yaml_path)
            except Exception as error:  # noqa: BLE001
                done, text = False, str(error)
            self.bridge.friction_done.emit(True, done, text)

        threading.Thread(target=work, daemon=True).start()

    def _read_motor_params(self) -> None:
        if self._arm_held():
            QMessageBox.warning(None, "读取电机参数", "请先停止当前模式，读参需要独占 CAN。")
            return
        left = self._left_can.strip()
        right = self._right_can.strip()
        bitrate = self._bitrate.strip()
        dbitrate = self._dbitrate.strip()
        if not left or not right or left == right:
            QMessageBox.warning(None, "读取电机参数", "左右臂需要两个不同的 CAN 口。")
            return
        if not bitrate.isdigit() or not dbitrate.isdigit():
            QMessageBox.warning(None, "读取电机参数", "波特率必须是整数。")
            return
        if (
            QMessageBox.question(
                None,
                "读取电机参数",
                "将查询双臂各关节 EEPROM 参数（不使能、不动臂）。继续？",
            )
            != QMessageBox.StandardButton.Yes
        ):
            return
        password = self._ask_sudo()
        if password is None:
            return
        self._configuration_busy = True
        self._emit_ui()
        self._motor_message = "正在读取电机参数…"
        self.motorInfoChanged.emit()
        self._set_status("正在读取电机参数…")
        threading.Thread(
            target=self._read_motor_params_worker,
            args=(left, right, bitrate, dbitrate, password),
            daemon=True,
        ).start()

    def _read_motor_params_worker(
        self, left: str, right: str, bitrate: str, dbitrate: str, password: str | None
    ) -> None:
        script = ROOT / "scripts" / "run_read_motor_params.sh"
        try:
            configure_can(left, right, bitrate, dbitrate, password)
            env = os.environ.copy()
            env["OPENARM_LEFT_FOLLOWER_CAN"] = left
            env["OPENARM_RIGHT_FOLLOWER_CAN"] = right
            result = subprocess.run(
                ["bash", str(script)],
                cwd=ROOT,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
            output = (result.stdout or "").strip()
            ok = result.returncode == 0
        except Exception as error:  # noqa: BLE001
            output = str(error)
            ok = False
        self.bridge.motor_done.emit(ok, output)

    def _read_motor_params_done(self, ok: bool, output: str) -> None:
        self._configuration_busy = False
        self._emit_ui()
        if output:
            shown = output if len(output) < 12000 else ("…\n" + output[-12000:])
            self._append_log(f"\n--- 电机参数 ---\n{shown}\n")
        report = ""
        csv_path = ""
        for line in output.splitlines():
            if line.startswith("REPORT="):
                report = line.split("=", 1)[1].strip()
            elif line.startswith("CSV="):
                csv_path = line.split("=", 1)[1].strip()
        self._motor_report = report
        if not ok:
            self._set_status("读取电机参数失败")
            self._motor_message = output.strip() or "读取失败"
            self.motorInfoChanged.emit()
            return
        if csv_path:
            try:
                rows = load_motor_rows(Path(csv_path))
            except OSError as error:
                self._motor_message = f"报告已写入，表格读取失败：{error}"
            else:
                self._motors.replace(rows)
                self._motor_message = f"已读取 {len(rows)} 个关节"
        else:
            self._motor_message = "已读取，但没有找到参数表"
            if not report:
                self._motor_report = str(ROOT / "recordings" / "motor_params")
        self._set_status("电机参数已读取")
        self.motorInfoChanged.emit()

    def _set_impedance_profile(self, name: str) -> None:
        if name not in IMPEDANCE_LABELS or self._impedance_busy:
            return
        if not self.impedanceLive:
            return
        self._impedance_busy = True
        self._emit_ui()
        threading.Thread(target=self._impedance_worker, args=(name,), daemon=True).start()

    def _impedance_worker(self, name: str) -> None:
        try:
            ok, output = run_ros2(["param", "set", "/openarm_real_arm", "impedance_profile", name])
        except Exception as error:  # noqa: BLE001
            ok, output = False, str(error)
        label = IMPEDANCE_LABELS.get(name, name)
        self.bridge.notice.emit(f"已切到{label}：{output}" if ok else f"切换失败：{output}")
        self.bridge.idle_refresh.emit()

    def _soft_stop_moveit(self) -> None:
        threading.Thread(target=self._soft_stop_worker, daemon=True).start()

    def _soft_stop_worker(self) -> None:
        try:
            ok, output = run_ros2(["service", "call", "/real_arm/soft_stop", "std_srvs/srv/Trigger"])
        except Exception as error:  # noqa: BLE001
            ok, output = False, str(error)
        self.bridge.notice.emit("已软急停" if ok else f"软急停失败：{output}")
        self.bridge.idle_refresh.emit()

    def _open_replay(self, source: str = "") -> None:
        if (
            self._running()
            or not self.btn_start.isEnabled()
            or (self._calib_proc is not None and self._calib_proc.poll() is None)
        ):
            QMessageBox.warning(
                None,
                "暂不能回放",
                "请先停止并释放正在运行的遥操、示教、零重力或标定控制器。",
            )
            return
        if not source and self.log_path.is_file():
            prefix = "[quest-ui] replay-ready session: "
            for line in reversed(self.log_path.read_text(encoding="utf-8", errors="replace").splitlines()):
                if line.startswith(prefix):
                    path = Path(line[len(prefix):].strip())
                    path = path if path.is_absolute() else ROOT / path
                    if path.is_dir():
                        source = str(path)
                    break
        self._replay_bridge.load(source, self._left_can.strip(), self._right_can.strip())
        self.setPage("replay")

    def _on_redo_button(self) -> None:
        self._expire_undo()
        self._record("undo" if self._undo_open() else "redo")

    def _record(self, kind: str, *, quiet: bool = False) -> bool:
        if not self._record_available():
            return False
        try:
            if kind == "start":
                name = self._batch_name.strip()
                if not name:
                    QMessageBox.warning(None, "录制", "先填写批次名称。")
                    return False
                send_record(f"start,{name}")
                self._undo_until = 0.0
                self._recording = True
                self._apply_record_state()
                if self._mode() == "zerog":
                    self._set_record_hint("正在记录拖动时的实测关节。拖完后点结束。")
                else:
                    self._set_record_hint(f"正在录制 {name}。做完点停止。")
                self._sync_catalog()
            elif kind == "stop":
                send_record("stop")
                self._recording = False
                if self._mode() == "zerog":
                    self._awaiting_label = False
                    self._can_redo = False
                    self._apply_record_state()
                    self._set_record_hint("已记下这段拖动。到回放页选择该片段。")
                else:
                    self._awaiting_label = True
                    self._can_redo = True
                    self._apply_record_state()
                    self._set_record_hint("已停止。这条算成功、失败，还是重录？")
                self._sync_catalog()
            elif kind == "success":
                send_record("success")
                self._awaiting_label = False
                self._can_redo = True
                self._apply_record_state()
                self._set_record_hint("已标为成功。可以录下一条，或重录刚刚这一条。")
                self._sync_catalog()
            elif kind == "fail":
                send_record("fail")
                self._awaiting_label = False
                self._can_redo = True
                self._apply_record_state()
                self._set_record_hint("已标为失败。可以录下一条，或重录刚刚这一条。")
                self._sync_catalog()
            elif kind == "redo":
                send_record("redo")
                self._undo_awaiting = self._awaiting_label
                self._awaiting_label = False
                self._recording = False
                self._can_redo = False
                self._undo_until = time.monotonic() + UNDO_SECONDS
                self._apply_record_state()
                self._set_record_hint(DISCARD_HINT)
                self._sync_catalog()
            elif kind == "undo":
                if not self._undo_open():
                    self._set_record_hint(UNDO_EXPIRED_HINT)
                    self._apply_record_state()
                    return False
                send_record("undo")
                self._awaiting_label = self._undo_awaiting
                self._can_redo = True
                self._undo_until = 0.0
                self._apply_record_state()
                self._set_record_hint(RESTORED_HINT)
                self._sync_catalog()
            return True
        except OSError as error:
            if quiet:
                self._set_record_hint(f"采集命令发送失败：{error}")
            else:
                QMessageBox.critical(None, "录制", str(error))
            return False


def create_engine(console: ConsoleController | None = None):
    console = console or ConsoleController()
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty("console", console)
    qml_root = Path(__file__).resolve().parent / "qml"
    engine.addImportPath(str(qml_root))
    engine.load(QUrl.fromLocalFile(str(qml_root / "Main.qml")))
    return engine, console


def run() -> None:
    script_dir = str(ROOT / "script")
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    app = QApplication(sys.argv)
    app.setApplicationName("OpenArm 遥操作")
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLE)
    _engine, _console = create_engine()
    if not _engine.rootObjects():
        sys.exit("无法加载操作台界面")
    sys.exit(app.exec())


MainWindow = ConsoleController

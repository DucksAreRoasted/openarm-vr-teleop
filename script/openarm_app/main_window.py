"""Qt main window for OpenArm teleop / MoveIt / zero-g control."""

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

from PySide6.QtCore import QObject, QSettings, Qt, QTimer, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtNetwork import QHostAddress, QUdpSocket
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QButtonGroup,
    QSizePolicy,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from lerobot_cameras import default_config, list_usb_cameras, load_specs, save_specs

from .data_manage_dialog import DataManageDialog
from .log_panel import IncrementalLogReader, LogPanel
from .gravity_urdf_seed import install_urdf_gravity_seed
from .gravity_identify import latest_two_sided_log, run_identification, summary as identify_summary
from .replay_dialog import ReplayDialog
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
from .theme import APP_STYLE, fit_window

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
# 行首锚定。启动动作结束后控制端打印这两句之一，见 openarm_quest_cartesian_control.cpp。
_QUEST_RUNNING_RE = re.compile(r"(?m)^\[quest-cartesian\] (?:bimanual )?running\.\s*$")


class WorkerBridge(QObject):
    finished_ok = Signal()
    finished_fail = Signal(str)
    calib_done = Signal(bool, str)
    calib_log = Signal(str)
    identify_done = Signal(bool, bool, str)
    motor_done = Signal(bool, str)
    idle = Signal(str)
    stop_stuck = Signal(str)
    notice = Signal(str)
    idle_refresh = Signal()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("OpenArm 遥操作")

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
        self._data_dialog: DataManageDialog | None = None
        self.bridge = WorkerBridge()
        self.bridge.finished_ok.connect(self._started)
        self.bridge.finished_fail.connect(self._fail)
        self.bridge.calib_done.connect(self._auto_calib_done)
        self.bridge.calib_log.connect(self._calib_log_line)
        self.bridge.identify_done.connect(self._identify_done)
        self.bridge.motor_done.connect(self._read_motor_params_done)
        self.bridge.idle.connect(self._idle)
        self.bridge.stop_stuck.connect(self._stop_stuck)
        self.bridge.notice.connect(self._notice)
        self.bridge.idle_refresh.connect(self._apply_impedance_state)
        self._calib_proc: subprocess.Popen[str] | None = None
        self._configuration_busy = False
        self._moveit_real = False

        self._log_reader = IncrementalLogReader()
        self._log_ready_tail = ""
        self._settings = QSettings("OpenArm", "TeleopConsole")
        self._log_expanded_height = self._settings.value("log/height", 260, type=int)
        self._build()
        fit_window(
            self,
            preferred_w=1280,
            preferred_h=900,
            min_w=960,
            min_h=620,
        )
        QTimer.singleShot(0, self._restore_log_layout)
        self._apply_mode()
        self._apply_record_state()
        self._refresh_usb()
        self.batch_edit.setText(datetime.now().strftime("session_%Y%m%d_%H%M%S"))

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
            self.record_hint.setText(
                f"手柄采集快捷键不可用：{capture_host}:{capture_port} 无法绑定。"
            )

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
        return self._running() or self.status.text() in ("正在启动…", "启动中…")

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
            return self._record(kind, quiet=True), self.record_hint.text()
        if action == "left-short" and not self._recording and not self.batch_edit.text().strip():
            return False, "请先在控制台填写批次名称。"
        kind = {
            "left-short": "stop" if self._recording else "start",
            "right-short": "success",
            "right-long": "fail",
        }[action]
        return self._record(kind, quiet=True), self.record_hint.text()

    def _undo_open(self) -> bool:
        return time.monotonic() < self._undo_until

    def _expire_undo(self) -> None:
        if self._undo_until and not self._undo_open():
            self._undo_until = 0.0
            self._apply_record_state()
            if self.record_hint.text() == DISCARD_HINT:
                self.record_hint.setText(UNDO_EXPIRED_HINT)

    def _build(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(12)

        head = QHBoxLayout()
        titles = QVBoxLayout()
        title = QLabel("OpenArm 遥操作")
        title.setObjectName("HeroTitle")
        sub = QLabel(
            "Quest 遥操 · MoveIt · 零重力 · 重力补偿微调"
        )
        sub.setObjectName("HeroSub")
        sub.setWordWrap(True)
        titles.addWidget(title)
        titles.addWidget(sub)
        head.addLayout(titles, stretch=1)
        self.status = QLabel("未启动")
        self.status.setObjectName("StatusWarn")
        self.status.setWordWrap(True)
        self.status.setMaximumWidth(320)
        self.status.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        head.addWidget(self.status)
        root.addLayout(head)

        workspace = QWidget()
        workspace.setObjectName("Workspace")
        columns = QHBoxLayout(workspace)
        columns.setContentsMargins(0, 0, 0, 0)
        columns.setSpacing(12)
        left = QVBoxLayout()
        left.setSpacing(8)
        right = QVBoxLayout()
        right.setSpacing(8)
        columns.addLayout(left, stretch=5)
        columns.addLayout(right, stretch=4)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(workspace)
        self.log_splitter = QSplitter(Qt.Orientation.Vertical)
        self.log_splitter.setChildrenCollapsible(False)
        self.log_splitter.setHandleWidth(7)
        self.log_splitter.addWidget(scroll)
        self.log_splitter.setStretchFactor(0, 1)
        root.addWidget(self.log_splitter, stretch=1)

        # Mode
        mode_box = QGroupBox("模式")
        mode_row = QGridLayout(mode_box)
        mode_row.setSpacing(16)
        self.mode_group = QButtonGroup(self)
        self.radio_sim = QRadioButton("仿真遥操")
        self.radio_real = QRadioButton("真机遥操")
        self.radio_moveit = QRadioButton("MoveIt")
        self.radio_zerog = QRadioButton("零重力")
        self.radio_zerog.setToolTip(
            "零重力：KDL G(q) + gravity_comp.yaml（关节补偿/摩擦等）"
        )
        self.radio_sim.setChecked(True)
        for i, r in enumerate(
            (
                self.radio_sim,
                self.radio_real,
                self.radio_moveit,
                self.radio_zerog,
            )
        ):
            self.mode_group.addButton(r, i)
            mode_row.addWidget(r, i // 4, i % 4)
        self.mode_group.idClicked.connect(lambda _: self._apply_mode())
        self.quest_profile = QComboBox()
        for title, value in (("宇树参考（默认）", "unitree_style"), ("PIKA 参考", "pika_style")):
            self.quest_profile.addItem(title, value)
        index = self.quest_profile.findData(os.environ.get("VR_TELEOP_MODE", "unitree_style"))
        self.quest_profile.setCurrentIndex(max(0, index))
        self._quest_profile_requested = self.quest_profile.currentData()
        self.quest_profile.setToolTip("宇树参考模式使用固定的机器人方向；先在仿真中核对移动方向、手腕姿态和活动范围。")
        profile_row = QHBoxLayout()
        profile_row.addWidget(QLabel("Quest 跟随方式"))
        profile_row.addWidget(self.quest_profile, 1)
        mode_row.addLayout(profile_row, 2, 0, 1, 4)
        left.addWidget(mode_box)

        impedance = QGroupBox("MoveIt 软硬")
        impedance_layout = QVBoxLayout(impedance)
        impedance_hint = QLabel(
            "启动前选择起始软硬。真机运行中可切换；软急停保持使能，停止并释放才会失能下落。"
        )
        impedance_hint.setObjectName("Hint")
        impedance_hint.setWordWrap(True)
        impedance_layout.addWidget(impedance_hint)
        start_row = QHBoxLayout()
        self.impedance_profile = QComboBox()
        for value, title in IMPEDANCE_LABELS.items():
            if value == "drag":
                continue
            self.impedance_profile.addItem(title, value)
        self.impedance_profile.setToolTip("只决定这次启动时的软硬。运行中用下面的按钮切换。")
        start_row.addWidget(QLabel("起始软硬"))
        start_row.addWidget(self.impedance_profile, 1)
        impedance_layout.addLayout(start_row)
        live_row = QHBoxLayout()
        self.impedance_buttons: dict[str, QPushButton] = {}
        for value, title in IMPEDANCE_LABELS.items():
            button = QPushButton(title)
            button.setToolTip(IMPEDANCE_TIPS[value] + " 切换大约 0.5 秒过渡。")
            button.clicked.connect(lambda _checked=False, name=value: self._set_impedance_profile(name))
            self.impedance_buttons[value] = button
            live_row.addWidget(button)
        self.btn_soft_stop = QPushButton("软急停")
        self.btn_soft_stop.setObjectName("Danger")
        self.btn_soft_stop.setToolTip(
            "取消当前轨迹并低刚度保持，不掉使能。之后可以手推；大范围移动再点「手拖」。"
        )
        self.btn_soft_stop.clicked.connect(self._soft_stop_moveit)
        live_row.addWidget(self.btn_soft_stop)
        impedance_layout.addLayout(live_row)
        self.impedance_box = impedance
        left.addWidget(impedance)

        # Connection — two-column CAN fields
        link = QGroupBox("连接")
        link_outer = QVBoxLayout(link)
        can_row = QHBoxLayout()
        form_l = QFormLayout()
        form_r = QFormLayout()
        form_l.setSpacing(8)
        form_r.setSpacing(8)
        self.left_can = QLineEdit(os.environ.get("OPENARM_LEFT_FOLLOWER_CAN", "can1"))
        self.right_can = QLineEdit(os.environ.get("OPENARM_RIGHT_FOLLOWER_CAN", "can0"))
        self.bitrate = QLineEdit(os.environ.get("OPENARM_CAN_BITRATE", "1000000"))
        self.dbitrate = QLineEdit(os.environ.get("OPENARM_CAN_DBITRATE", "5000000"))
        for edit in (self.left_can, self.right_can, self.bitrate, self.dbitrate):
            edit.setMaximumWidth(160)
        form_l.addRow("左臂 CAN", self.left_can)
        form_l.addRow("波特率", self.bitrate)
        form_r.addRow("右臂 CAN", self.right_can)
        form_r.addRow("数据波特率", self.dbitrate)
        can_row.addLayout(form_l)
        can_row.addLayout(form_r)
        can_row.addStretch(1)
        link_outer.addLayout(can_row)
        self.safety = QCheckBox(
            "确认 CAN 独占、工作区清空、急停可用"
        )
        self.safety.setToolTip("真机操作前确认；MoveIt 和零重力连接真机也必须勾选。")
        self.safety.toggled.connect(lambda _: self._apply_mode())
        link_outer.addWidget(self.safety)
        self.quest_label = QLabel(f"Quest：https://{local_ip()}:{HTTP_PORT}")
        self.quest_label.setObjectName("Hint")
        self.quest_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        link_outer.addWidget(self.quest_label)
        left.addWidget(link)

        # Cameras
        cams = QGroupBox("相机 USB")
        cam_layout = QFormLayout(cams)
        cam_layout.setSpacing(8)
        self.camera_boxes: dict[str, QComboBox] = {}
        for name, cam_title in (("head", "头部"), ("wrist_left", "左腕"), ("wrist_right", "右腕")):
            box = QComboBox()
            box.addItem("（不选）")
            box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            self.camera_boxes[name] = box
            cam_layout.addRow(cam_title, box)
        cam_btns = QHBoxLayout()
        self.btn_refresh_usb = QPushButton("刷新 USB")
        self.btn_open_cams = QPushButton("打开相机预览")
        self.btn_refresh_usb.clicked.connect(self._refresh_usb)
        self.btn_open_cams.clicked.connect(self._open_cameras)
        cam_btns.addWidget(self.btn_refresh_usb)
        cam_btns.addWidget(self.btn_open_cams)
        cam_btns.addStretch(1)
        cam_layout.addRow(cam_btns)
        self.camera_status = QLabel("刷新后为头部 / 左腕 / 右腕选择 USB 口")
        self.camera_status.setObjectName("Hint")
        cam_layout.addRow(self.camera_status)
        left.addWidget(cams)

        # Actions
        actions = QFrame()
        action_row = QHBoxLayout(actions)
        action_row.setContentsMargins(0, 4, 0, 0)
        self.btn_start = QPushButton("启动")
        self.btn_start.setObjectName("Primary")
        self.btn_stop = QPushButton("停止并释放")
        self.btn_stop.setObjectName("Danger")
        self.btn_motor = QPushButton("读取电机参数")
        self.btn_start.clicked.connect(self._start)
        self.btn_stop.clicked.connect(self._stop)
        self.btn_motor.clicked.connect(self._read_motor_params)
        action_row.addWidget(self.btn_start)
        action_row.addWidget(self.btn_stop)
        action_row.addWidget(self.btn_motor)
        action_row.addStretch(1)
        root.addWidget(actions)
        left.addStretch(1)

        # Gravity
        gravity = QGroupBox("重力补偿")
        g_layout = QVBoxLayout(gravity)
        g_hint = QLabel(
            "推荐顺序：① URDF 初值（新臂或重新开始时）→ ② 自动微调（动臂采数）→ "
            "③ 惯量标定（用②的日志辨识质量/质心）→ 零重力验证 → 再次自动微调精修。"
            "各步骤直接覆盖当前配置，不另存备份。"
        )
        g_hint.setObjectName("Hint")
        g_hint.setWordWrap(True)
        g_layout.addWidget(g_hint)
        g_btns = QHBoxLayout()
        self.btn_seed = QPushButton("使用 URDF 初值")
        self.btn_seed.setToolTip(
            "不动臂：把质量/质心、关节倍率和偏置恢复为 URDF 模型基准，前馈增益设为 0.95。\n"
            "保留夹爪/相机安装状态、负载和摩擦；直接覆盖当前配置，下次启动生效。"
        )
        self.btn_calib = QPushButton("自动微调")
        self.btn_calib.setObjectName("Primary")
        self.btn_calib.setToolTip(
            "机械臂会运动（约 10–15 分钟）：腕→肘→肩各姿态从 ±δ 两侧软持采数，日志供惯量标定使用。\n"
            "随后在当前配置上小幅调整关节倍率/偏置，留出姿态验证改善才写入；不改质量/质心和摩擦。"
        )
        self.btn_identify = QPushButton("惯量标定")
        self.btn_identify.setToolTip(
            "不动臂：用最近一次自动微调的两侧日志辨识各连杆质量/质心，远端增益归 1。\n"
            "先预览误差对比，确认后直接覆盖当前配置，下次启动生效。"
        )
        self.btn_seed.clicked.connect(self._seed_gravity_from_urdf)
        self.btn_calib.clicked.connect(self._auto_calib_gravity)
        self.btn_identify.clicked.connect(self._identify_gravity)
        g_btns.addWidget(self.btn_seed)
        g_btns.addWidget(self.btn_calib)
        g_btns.addWidget(self.btn_identify)
        g_btns.addStretch(1)
        g_layout.addLayout(g_btns)
        g_actions = QLabel(
            "URDF 初值、惯量标定只写配置、不动臂；自动微调会驱动机械臂。写入后下次启动零重力/遥操生效。"
        )
        g_actions.setObjectName("Hint")
        g_actions.setWordWrap(True)
        g_layout.addWidget(g_actions)
        right.addWidget(gravity)

        # Recording
        record = QGroupBox("数据采集")
        r_layout = QVBoxLayout(record)
        record_header = QHBoxLayout()
        record_header.addWidget(QLabel("录制 → 停止 → 标注"))
        record_header.addStretch()
        self.record_badge = QLabel("未就绪")
        self.record_badge.setObjectName("RecordBadge")
        record_header.addWidget(self.record_badge)
        r_layout.addLayout(record_header)
        batch_row = QHBoxLayout()
        batch_row.addWidget(QLabel("批次"))
        self.batch_edit = QLineEdit()
        self.batch_edit.setPlaceholderText("为这批动作命名")
        batch_row.addWidget(self.batch_edit, stretch=1)
        r_layout.addLayout(batch_row)
        row1 = QHBoxLayout()
        self.rec_start = QPushButton("开始录制")
        self.rec_start.setObjectName("Primary")
        self.rec_stop = QPushButton("结束本条")
        self.rec_start.clicked.connect(lambda: self._record("start"))
        self.rec_stop.clicked.connect(lambda: self._record("stop"))
        row1.addWidget(self.rec_start)
        row1.addWidget(self.rec_stop)
        r_layout.addLayout(row1)
        row2 = QHBoxLayout()
        self.rec_ok = QPushButton("标记成功")
        self.rec_ok.setObjectName("Success")
        self.rec_bad = QPushButton("标记失败")
        self.rec_bad.setObjectName("Danger")
        self.rec_redo = QPushButton("丢弃重录")
        self.rec_ok.clicked.connect(lambda: self._record("success"))
        self.rec_bad.clicked.connect(lambda: self._record("fail"))
        self.rec_redo.clicked.connect(self._on_redo_button)
        row2.addWidget(self.rec_ok)
        row2.addWidget(self.rec_bad)
        row2.addWidget(self.rec_redo)
        r_layout.addLayout(row2)
        manage_row = QHBoxLayout()
        self.btn_data_manage = QPushButton("数据管理…")
        self.btn_data_manage.clicked.connect(self._open_data_manage)
        manage_row.addWidget(self.btn_data_manage)
        self.btn_replay = QPushButton("动作回放…")
        self.btn_replay.setToolTip("Quest 遥操或手拖示教录制后，停止并释放，再选择片段自动回放；支持仿真、变速、暂停和停止。")
        self.btn_replay.clicked.connect(lambda: self._open_replay())
        manage_row.addWidget(self.btn_replay)
        r_layout.addLayout(manage_row)
        self.record_hint = QLabel("启动后可录；停录后再标成功 / 失败 / 重录")
        self.record_hint.setObjectName("Hint")
        self.record_hint.setWordWrap(True)
        r_layout.addWidget(self.record_hint)
        shortcuts = QFrame()
        shortcuts.setObjectName("ShortcutCard")
        shortcut_grid = QGridLayout(shortcuts)
        for column, (hand, short, long) in enumerate((
            ("左摇杆", "短按  开始 / 结束录制", "按住 1.5 秒  丢弃当前条"),
            ("右摇杆 · 停录后", "短按  标记成功", "按住 1.5 秒  标记失败"),
        )):
            heading = QLabel(hand)
            heading.setObjectName("ShortcutTitle")
            shortcut_grid.addWidget(heading, 0, column)
            for row, text in enumerate((short, long), 1):
                label = QLabel(text)
                label.setWordWrap(True)
                shortcut_grid.addWidget(label, row, column)
        note = QLabel(
            "按住时约每 0.3 秒轻振一下，满 1.5 秒即执行，中途松开则取消。"
            "丢弃后 10 秒内可再长按左摇杆或点按钮撤销。A/B、X/Y 控制夹爪"
        )
        note.setObjectName("Hint")
        note.setWordWrap(True)
        shortcut_grid.addWidget(note, 3, 0, 1, 2)
        r_layout.addWidget(shortcuts)
        right.addWidget(record)
        right.removeWidget(gravity)
        right.addWidget(gravity)

        self.btn_refresh_usb.setToolTip('重新扫描 USB 相机设备，更新设备列表。')
        self.btn_open_cams.setToolTip('打开相机预览窗口，检查画面和相机选择。')
        self.btn_start.setToolTip('启动当前选择的模式；真机模式需先完成安全确认。')
        self.btn_stop.setToolTip('停止当前运行模式并释放设备；自动微调期间请使用硬件急停中止运动。')
        self.btn_motor.setToolTip('读取双臂电机参数并显示报告；不使能电机、不运动，需要独占 CAN。')
        self.rec_start.setToolTip('开始记录当前遥操作数据。')
        self.rec_stop.setToolTip('结束本段录制，随后选择成功、失败或重新录制。')
        self.rec_ok.setToolTip('将刚结束的录制标记为成功并保存。')
        self.rec_bad.setToolTip('将刚结束的录制标记为失败并保存。')
        self.rec_redo.setToolTip('把刚刚这一条移到回收站，下次开始复用同一个序号。10 秒内可撤销。')
        self.btn_data_manage.setToolTip('浏览已录制数据，查看标签并管理或导出记录。')

        # A shared, bounded console below the controls; never replace its contents.
        archive = Path(tempfile.gettempdir()) / (
            f"openarm_console_{datetime.now():%Y%m%d_%H%M%S_%f}_{os.getpid()}.log"
        )
        self.log_panel = LogPanel(self, settings=self._settings, archive_path=archive)
        self.log = self.log_panel.view
        self.log_splitter.addWidget(self.log_panel)
        self.log_splitter.setStretchFactor(1, 0)
        self.log_splitter.splitterMoved.connect(self._remember_log_height)
        self.log_panel.layout_changed.connect(self._resize_log_panel)
        self.log_panel.fatal_received.connect(self._log_fatal)
        self._log_toggle_shortcut = QShortcut(QKeySequence("Ctrl+J"), self)
        self._log_toggle_shortcut.activated.connect(
            lambda: self.log_panel.set_expanded(not self.log_panel.toggle.isChecked())
        )
        right.addStretch(1)

    def _restore_log_layout(self) -> None:
        state = self._settings.value("log/splitterState")
        if state:
            self.log_splitter.restoreState(state)
            sizes = self.log_splitter.sizes()
            if sizes[1] > 32:
                self._log_expanded_height = sizes[1]
        else:
            self._log_expanded_height = min(320, max(200, int(self.height() * .3)))
        self._resize_log_panel()

    def _remember_log_height(self, *_args) -> None:
        if self.log_panel.body.parentWidget() is self.log_panel and self.log_panel.toggle.isChecked():
            self._log_expanded_height = self.log_splitter.sizes()[1]

    def _resize_log_panel(self) -> None:
        height = self.log_splitter.height()
        docked = self.log_panel.body.parentWidget() is self.log_panel
        if self.log_panel.toggle.isChecked() and docked:
            maximum = max(120, int(height * .6))
            self.log_panel.setMaximumHeight(maximum)
            wanted = min(maximum, max(120, self._log_expanded_height))
        else:
            wanted = 32
        self.log_splitter.setSizes([max(1, height - wanted), wanted])

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        if hasattr(self, "log_panel"):
            self._resize_log_panel()

    def _log_fatal(self, text: str) -> None:
        self.status.setStyleSheet("color: #ff6b6b;")
        self.status.setText("严重错误：" + text[:100])

    def _mode(self) -> str:
        if self.radio_real.isChecked():
            return "real"
        if self.radio_moveit.isChecked():
            return "moveit"
        if self.radio_zerog.isChecked():
            return "zerog"
        return "sim"

    def _hardware_mode(self, mode: str | None = None) -> bool:
        mode = mode or self._mode()
        if mode in ("real", "zerog"):
            return True
        if mode == "moveit":
            return self.safety.isChecked()
        return False

    def _running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _record_available(self) -> bool:
        mode = self._mode()
        if not self._running() or mode in ("moveit", "zerog"):
            return False
        if mode in ("sim", "real"):
            return self._quest_ready
        return False

    def _set_can_enabled(self, enabled: bool) -> None:
        for w in (
            self.left_can,
            self.right_can,
            self.bitrate,
            self.dbitrate,
            self.safety,
        ):
            w.setEnabled(enabled)

    def _apply_mode(self) -> None:
        mode = self._mode()
        self.quest_profile.setEnabled(mode in ("sim", "real") and not self._running())
        can_ui = mode in ("real", "moveit", "zerog")
        if not self._running():
            self._set_can_enabled(can_ui)
            if mode == "sim":
                self.safety.setChecked(False)
            if mode == "moveit":
                self.status.setText(
                    "MoveIt 实机（已勾选急停）"
                    if self.safety.isChecked()
                    else "MoveIt 仿真（未勾选急停）"
                )
            elif mode == "zerog":
                self.status.setText("零重力：勾选急停后启动（含关节补偿/摩擦）")
            elif self.status.text().startswith(("MoveIt", "零重力")):
                self.status.setText("未启动")
        self._apply_record_state()
        self._apply_impedance_state()

    def _apply_impedance_state(self) -> None:
        moveit = self._mode() == "moveit"
        running = self._running()
        self.impedance_box.setVisible(moveit)
        self.impedance_profile.setEnabled(moveit and not running)
        live = moveit and running and self._moveit_real
        for button in self.impedance_buttons.values():
            button.setEnabled(live)
        self.btn_soft_stop.setEnabled(live)

    def _notice(self, text: str) -> None:
        self.status.setText(text[:120])
        self._append_log(text if text.endswith("\n") else text + "\n")

    def _set_impedance_profile(self, name: str) -> None:
        for button in self.impedance_buttons.values():
            button.setEnabled(False)
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
        self.btn_soft_stop.setEnabled(False)
        threading.Thread(target=self._soft_stop_worker, daemon=True).start()

    def _soft_stop_worker(self) -> None:
        try:
            ok, output = run_ros2(["service", "call", "/real_arm/soft_stop", "std_srvs/srv/Trigger"])
        except Exception as error:  # noqa: BLE001
            ok, output = False, str(error)
        self.bridge.notice.emit("已软急停" if ok else f"软急停失败：{output}")
        self.bridge.idle_refresh.emit()

    def _apply_record_state(self) -> None:
        running = self._running()
        self.quest_profile.setEnabled(self._mode() in ("sim", "real") and not running)
        self.btn_replay.setEnabled(not running)
        for r in (
            self.radio_sim,
            self.radio_real,
            self.radio_moveit,
            self.radio_zerog,
        ):
            r.setEnabled(not running)
        self.btn_start.setEnabled(not running and not self._configuration_busy)
        self.btn_stop.setEnabled(running)
        self.btn_calib.setEnabled(not running and not self._configuration_busy)
        self.btn_seed.setEnabled(not running and not self._configuration_busy)
        self.btn_identify.setEnabled(not running and not self._configuration_busy)
        self.btn_motor.setEnabled(not running and not self._configuration_busy)
        self._apply_impedance_state()
        if running:
            self._set_can_enabled(False)
        elif self._mode() in ("real", "moveit", "zerog"):
            self._set_can_enabled(True)
        teleop = self._record_available()
        badge, tone = (("录制中", "recording") if self._recording else
                       ("待标注", "pending") if self._awaiting_label else
                       ("可以录制", "ready") if teleop else ("未就绪", "idle"))
        self.record_badge.setText(badge)
        self.record_badge.setProperty("tone", tone)
        self.record_badge.style().unpolish(self.record_badge)
        self.record_badge.style().polish(self.record_badge)
        self.rec_start.setEnabled(teleop and not self._awaiting_label and not self._recording)
        self.rec_stop.setEnabled(teleop and self._recording)
        self.rec_ok.setEnabled(teleop and self._awaiting_label)
        self.rec_bad.setEnabled(teleop and self._awaiting_label)
        undo_open = self._undo_open()
        self.rec_redo.setText("撤销丢弃" if undo_open else "丢弃重录")
        self.rec_redo.setEnabled(
            teleop and not self._recording and (self._can_redo or undo_open)
        )
        self.batch_edit.setEnabled(teleop and not self._awaiting_label and not self._recording)

    def _append_log(self, text: str, source: str = "控制台") -> None:
        self.log_panel.enqueue(text, source)

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
        for name, box in self.camera_boxes.items():
            current = box.currentText()
            box.blockSignals(True)
            box.clear()
            box.addItems(values)
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
            box.setCurrentText(pick)
            box.blockSignals(False)
        if cameras:
            self.camera_status.setText(f"读到 {len(cameras)} 个 Orbbec USB 口。选好后再开窗口。")
        else:
            self.camera_status.setText("没有读到 Orbbec 相机。确认 Gemini 335 已插入后再刷新。")

    def _save_camera_choice(self) -> bool:
        if not self._usb_by_label:
            return True
        chosen = {}
        used: set[str] = set()
        for name, box in self.camera_boxes.items():
            cam = self._usb_by_label.get(box.currentText())
            if cam is not None:
                if cam.port in used:
                    QMessageBox.warning(self, "相机", "头部、左腕、右腕不能选同一个 USB 口。")
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
                self, "相机", "相机采集已运行（电脑预览或后台采集）。如需更换 USB 口或打开电脑预览窗口，请先停止遥操并关闭已有相机窗口，再重新打开。"
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
            self.status.setText("三个相机窗口已打开")

    def _ask_sudo(self) -> str | None:
        if sudo_is_cached():
            return ""
        text, ok = QInputDialog.getText(
            self,
            "配置真机 CAN",
            "请输入当前用户的 sudo 密码。\n密码只传给这一次 CAN 配置，不会保存。",
            QLineEdit.EchoMode.Password,
        )
        if not ok:
            return None
        if not text:
            QMessageBox.warning(self, "配置真机 CAN", "sudo 密码不能为空。")
            return None
        return text

    def _start(self) -> None:
        if self._running() or self._configuration_busy:
            return
        mode = self._mode()
        if mode in ("sim", "real") and any(
            self._usb_by_label.get(box.currentText()) is not None
            for box in self.camera_boxes.values()
        ):
            if not self._save_camera_choice():
                return
        real = self._hardware_mode(mode)
        left = self.left_can.text().strip()
        right = self.right_can.text().strip()
        bitrate = self.bitrate.text().strip()
        dbitrate = self.dbitrate.text().strip()
        if real:
            title = {
                "moveit": "MoveIt 实机",
                "zerog": "零重力",
                "real": "真机遥操",
            }.get(mode, "真机")
            if not self.safety.isChecked():
                QMessageBox.warning(self, title, "启动前请先勾选安全确认。")
                return
            if not left or not right or left == right:
                QMessageBox.warning(self, title, "左右臂需要两个不同的 CAN 口。")
                return
            if not bitrate.isdigit() or not dbitrate.isdigit():
                QMessageBox.warning(self, title, "波特率必须是整数。")
                return
        password = None
        if real:
            password = self._ask_sudo()
            if password is None:
                return
        if mode == "moveit":
            self.status.setText("正在启动 MoveIt 实机…" if real else "正在启动 MoveIt 仿真…")
        elif mode == "zerog":
            self.status.setText("正在启动零重力…")
        else:
            self.status.setText("正在启动…")
        self.btn_start.setEnabled(False)
        self._configuration_busy = True
        self.btn_seed.setEnabled(False)
        self.btn_identify.setEnabled(False)
        self._quest_profile_requested = self.quest_profile.currentData()
        self._moveit_profile_requested = self.impedance_profile.currentData()
        self.quest_profile.setEnabled(False)
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
        environment["OPENARM_LEFT_FOLLOWER_CAN"] = left
        environment["OPENARM_RIGHT_FOLLOWER_CAN"] = right
        environment.setdefault("OPENARM_ROOT", str(ROOT))
        environment.setdefault("OPENARM_GRAVITY_YAML", str(ROOT / "config" / "gravity_comp.yaml"))
        self._log_reader.reset()
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
            environment["VR_TELEOP_MODE"] = self._quest_profile_requested
        self.proc = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def _started(self) -> None:
        self.status.setStyleSheet("")
        self._configuration_busy = False
        mode = self._mode()
        if mode == "moveit":
            kind = "MoveIt 实机" if self.safety.isChecked() else "MoveIt 仿真"
        elif mode == "zerog":
            kind = "零重力"
        else:
            kind = {"real": "真机", "sim": "仿真"}.get(mode, "仿真")
        if mode in ("sim", "real"):
            self.status.setText("启动中…")
        else:
            self.status.setText(f"{kind}运行中")
        if mode == "zerog":
            self.record_hint.setText("零重力已托住双臂。可用「重力」分区微调；不录遥操数据。")
        elif mode == "moveit" and self.safety.isChecked():
            self._moveit_real = True
            self.record_hint.setText("MoveIt 实机：可用软硬按钮或软急停。停止并释放会失能下落。")
        elif mode == "moveit":
            self.record_hint.setText("MoveIt 仿真只在 RViz 里动模型，不录遥操数据。")
        elif mode in ("sim", "real"):
            self.record_hint.setText(QUEST_NOT_READY_MESSAGE)
        else:
            self.record_hint.setText("可以开始录。停下来之后选择成功、失败或重录。")
        self._awaiting_label = False
        self._recording = False
        self._can_redo = False
        self._undo_until = 0.0
        self._apply_record_state()
        self._poll_timer.start()
        self._sync_data_dialog()
        if mode in ("sim", "real"):
            self._refresh_log()

    def _quest_ready_hint(self) -> str:
        return "可以开始录。停下来之后选择成功、失败或重录。"

    def _fail(self, text: str) -> None:
        self._configuration_busy = False
        self._quest_ready = False
        self._undo_until = 0.0
        self._moveit_real = False
        self.status.setText("启动失败")
        self.btn_start.setEnabled(True)
        self._apply_mode()
        QMessageBox.critical(self, "启动失败", text)

    def _stop(self) -> None:
        proc = self.proc
        if proc is None or proc.poll() is not None:
            return
        self.status.setText("正在释放…")
        self.btn_stop.setEnabled(False)

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
        self.status.setText(text)
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
        self.status.setText(text)
        self.record_hint.setText("启动之后才能录。停下来再标成功、失败，或重录。")
        self._poll_timer.stop()
        self._apply_mode()
        self._apply_record_state()
        self._sync_data_dialog()

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
                self.status.setText(f"{kind}运行中")
                self.record_hint.setText(self._quest_ready_hint())
                self._apply_record_state()
                self._sync_data_dialog()
        if text:
            self.log_panel.enqueue(text, "控制器")

    def _close_log(self) -> None:
        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None

    def _identify_gravity(self) -> None:
        title = "惯量标定"
        if self._running() or self._configuration_busy or not self.btn_start.isEnabled() or self._calib_proc is not None:
            QMessageBox.warning(self, title, "请先停止控制器，并等待当前操作完成。")
            return
        log_path = latest_two_sided_log()
        if log_path is None:
            QMessageBox.warning(
                self, title,
                "没有找到两侧接近的标定日志。\n请先运行一次「自动微调」采集数据，再做惯量标定。",
            )
            return
        self._identify_log = log_path
        self._start_identification(log_path, write=False)

    def _start_identification(self, log_path: Path, write: bool) -> None:
        yaml_path = resolve_gravity_yaml()
        self._configuration_busy = True
        for button in (self.btn_identify, self.btn_seed, self.btn_calib, self.btn_start):
            button.setEnabled(False)
        self.status.setText("惯量标定写入中…" if write else "惯量标定计算中…")
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
        for button in (self.btn_identify, self.btn_seed, self.btn_calib, self.btn_start):
            button.setEnabled(True)
        title = "惯量标定"
        self._append_log(output + "\n", "标定")
        if not ok:
            self.status.setText(f"{title}失败")
            shown = output if len(output) < 4000 else ("…\n" + output[-4000:])
            QMessageBox.critical(self, title, shown)
            return
        if write:
            self.status.setText("惯量标定已写入；下次启动生效")
            QMessageBox.information(
                self, title,
                "已写入辨识结果，下次启动生效。\n"
                "下一步：零重力下逐姿态验证左右是否一致，再用「自动微调」小幅精修。",
            )
            return
        self.status.setText("惯量标定预览完成，未写入")
        if QMessageBox.question(
            self, title,
            f"日志：{self._identify_log.name}\n留出姿态误差 RMS (Nm, j1..j7)：\n\n"
            f"{identify_summary(output)}\n\n"
            "写入后将用辨识的连杆质量/质心替换当前修正参数（远端增益、倍率、偏置归一），"
            "直接覆盖当前配置，下次启动生效。\n"
            "若出现「已夹限」，建议先检查夹爪/相机安装状态再决定。\n是否写入？",
        ) == QMessageBox.StandardButton.Yes:
            self._start_identification(self._identify_log, write=True)

    def _seed_gravity_from_urdf(self) -> None:
        title = "使用 URDF 初值"
        if self._running() or self._configuration_busy or not self.btn_start.isEnabled() or self._calib_proc is not None:
            QMessageBox.warning(self, title, "请先停止控制器，并等待当前操作完成。")
            return
        yaml_path = resolve_gravity_yaml()
        urdf_path = Path(os.environ.get("OPENARM_TELEOP_URDF", str(ROOT / "assets" / "v1.urdf"))).expanduser()
        if not urdf_path.is_absolute():
            urdf_path = ROOT / urdf_path
        if QMessageBox.question(
            self,
            title,
            "将当前重力补偿替换为 URDF 模型初值？\n\n"
            "质量/质心、逐关节倍率和偏置将恢复为模型基准；前馈增益设为 0.95。\n"
            "夹爪/相机安装状态、负载及摩擦参数会保留；当前调好的参数将被直接覆盖。\n"
            "此操作只写配置，不会驱动机械臂；下次启动生效。\n"
            "下一步：运行「自动微调」采数，再做「惯量标定」。\n\n"
            f"URDF：{urdf_path}\n配置：{yaml_path}",
        ) != QMessageBox.StandardButton.Yes:
            return
        self.btn_seed.setEnabled(False)
        try:
            output = install_urdf_gravity_seed(yaml_path, urdf_path)
        except Exception as error:  # noqa: BLE001
            self.status.setText("URDF 初值设置失败")
            QMessageBox.critical(self, title, str(error))
            return
        finally:
            self.btn_seed.setEnabled(True)
        self._append_log(f"\n--- URDF 初值 ---\n{output}\n", "标定")
        self.status.setText("URDF 初值已写入；下次启动生效")
        QMessageBox.information(
            self, title, "已写入 URDF 初值，下次启动生效。\n下一步：运行「自动微调」采数，再做「惯量标定」。"
        )

    def _auto_calib_gravity(self) -> None:
        title = "自动微调"
        if self._running() or self._configuration_busy:
            QMessageBox.warning(self, title, "请先停止当前模式，标定需要独占 CAN。")
            return
        if self._calib_proc is not None and self._calib_proc.poll() is None:
            QMessageBox.warning(self, title, "标定正在进行中。")
            return
        if not self.safety.isChecked():
            QMessageBox.warning(self, title, "标定前请勾选安全确认。")
            return
        left = self.left_can.text().strip()
        right = self.right_can.text().strip()
        bitrate = self.bitrate.text().strip()
        dbitrate = self.dbitrate.text().strip()
        if not left or not right or left == right:
            QMessageBox.warning(self, title, "左右臂需要两个不同的 CAN 口。")
            return
        if not bitrate.isdigit() or not dbitrate.isdigit():
            QMessageBox.warning(self, title, "波特率必须是整数。")
            return
        yaml_path = resolve_gravity_yaml()
        if not yaml_path.is_file():
            QMessageBox.warning(self, title, f"找不到当前重力配置：{yaml_path}")
            return
        if (
            QMessageBox.question(
                self,
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
        self.btn_calib.setEnabled(False)
        self._configuration_busy = True
        self.btn_seed.setEnabled(False)
        self.btn_identify.setEnabled(False)
        self.btn_motor.setEnabled(False)
        self.btn_start.setEnabled(False)
        self.status.setText("正在自动微调…（日志区有实时输出）")
        self._append_log("\n--- 自动微调开始 ---\n", "标定")
        threading.Thread(
            target=self._auto_calib_worker,
            args=(left, right, bitrate, dbitrate, password, yaml_path),
            daemon=True,
        ).start()

    def _auto_calib_worker(
        self, left: str, right: str, bitrate: str, dbitrate: str, password: str | None,
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
            self.status.setText(line.strip()[:120])

    def _auto_calib_done(self, ok: bool, output: str) -> None:
        self.log_panel.enqueue("", "标定", final=True)
        self.log_panel.flush()
        self._configuration_busy = False
        self.btn_calib.setEnabled(True)
        self.btn_seed.setEnabled(True)
        self.btn_identify.setEnabled(True)
        self.btn_start.setEnabled(True)
        self.btn_motor.setEnabled(True)
        title = "自动微调"
        if not ok:
            self.status.setText(f"{title}失败")
            shown = output if len(output) < 4000 else ("…\n" + output[-4000:])
            QMessageBox.critical(self, title, shown or "失败")
            return
        self.status.setText(f"{title}完成")
        QMessageBox.information(
            self, title,
            "已完成关节倍率/偏置微调与独立姿态验证；未改善的关节保留原值。\n"
            "从 URDF 初值开始时：下一步点「惯量标定」，用本次日志辨识质量/质心。\n"
            "已做过惯量标定时：启动「零重力」检查手感。",
        )

    def _read_motor_params(self) -> None:
        if self._running() or self._configuration_busy:
            QMessageBox.warning(self, "读取电机参数", "请先停止当前模式，读参需要独占 CAN。")
            return
        left = self.left_can.text().strip()
        right = self.right_can.text().strip()
        bitrate = self.bitrate.text().strip()
        dbitrate = self.dbitrate.text().strip()
        if not left or not right or left == right:
            QMessageBox.warning(self, "读取电机参数", "左右臂需要两个不同的 CAN 口。")
            return
        if not bitrate.isdigit() or not dbitrate.isdigit():
            QMessageBox.warning(self, "读取电机参数", "波特率必须是整数。")
            return
        if (
            QMessageBox.question(
                self,
                "读取电机参数",
                "将查询双臂各关节 EEPROM 参数（不使能、不动臂）。继续？",
            )
            != QMessageBox.StandardButton.Yes
        ):
            return
        password = self._ask_sudo()
        if password is None:
            return
        self.btn_motor.setEnabled(False)
        self._configuration_busy = True
        self.btn_start.setEnabled(False)
        self.btn_calib.setEnabled(False)
        self.btn_seed.setEnabled(False)
        self.btn_identify.setEnabled(False)
        self.status.setText("正在读取电机参数…")
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
        self.btn_motor.setEnabled(True)
        self.btn_start.setEnabled(True)
        running = self._running()
        self.btn_calib.setEnabled(not running)
        self.btn_seed.setEnabled(not running)
        self.btn_identify.setEnabled(not running)
        if output:
            shown = output if len(output) < 12000 else ("…\n" + output[-12000:])
            self._append_log(f"\n--- 电机参数 ---\n{shown}\n")
        if not ok:
            self.status.setText("读取电机参数失败")
            QMessageBox.critical(self, "读取电机参数", output or "失败")
            return
        report = ""
        for line in output.splitlines():
            if line.startswith("REPORT="):
                report = line.split("=", 1)[1].strip()
        self.status.setText("电机参数已读取")
        QMessageBox.information(
            self,
            "读取电机参数",
            "已读完并写入报告。\n"
            + (f"报告：{report}" if report else f"目录：{ROOT / 'recordings' / 'motor_params'}"),
        )

    def _sync_data_dialog(self) -> None:
        if self._data_dialog is None:
            return
        teleop = self._record_available()
        self._data_dialog.set_teleop_state(
            teleop_running=teleop,
            recording_busy=self._recording,
            awaiting_label=self._awaiting_label,
            current_batch=self.batch_edit.text().strip(),
        )
        self._data_dialog.refresh()

    def _open_data_manage(self) -> None:
        teleop = self._record_available()
        if self._data_dialog is None:
            self._data_dialog = DataManageDialog(
                self,
                current_batch=self.batch_edit.text().strip(),
                teleop_running=teleop,
                recording_busy=self._recording,
                awaiting_label=self._awaiting_label,
            )
            self._data_dialog.replay_requested.connect(self._open_replay)
        else:
            self._sync_data_dialog()
        self._data_dialog.show()
        self._data_dialog.raise_()
        self._data_dialog.activateWindow()

    def _open_replay(self, source: str = "") -> None:
        if self._running() or not self.btn_start.isEnabled() or (self._calib_proc is not None and self._calib_proc.poll() is None):
            QMessageBox.warning(self, "暂不能回放", "请先停止并释放正在运行的遥操、示教、零重力或标定控制器。")
            return
        if not source and self.log_path.is_file():
            prefix = "[quest-ui] replay-ready session: "
            for line in reversed(self.log_path.read_text(encoding="utf-8", errors="replace").splitlines()):
                if line.startswith(prefix):
                    path = Path(line[len(prefix):].strip())
                    path = path if path.is_absolute() else ROOT / path
                    if path.is_dir(): source = str(path)
                    break
        dialog = ReplayDialog(self, source=source, left_can=self.left_can.text().strip(),
                              right_can=self.right_can.text().strip())
        dialog.exec()

    def _on_redo_button(self) -> None:
        self._expire_undo()
        self._record("undo" if self._undo_open() else "redo")

    def _record(self, kind: str, *, quiet: bool = False) -> bool:
        if not self._record_available():
            return False
        try:
            if kind == "start":
                name = self.batch_edit.text().strip()
                if not name:
                    QMessageBox.warning(self, "录制", "先填写批次名称。")
                    return False
                send_record(f"start,{name}")
                self._undo_until = 0.0
                self._recording = True
                self._apply_record_state()
                self.rec_start.setEnabled(False)
                self.rec_stop.setEnabled(True)
                self.batch_edit.setEnabled(False)
                self.record_hint.setText(f"正在录制 {name}。做完点停止。")
                self._sync_data_dialog()
            elif kind == "stop":
                send_record("stop")
                self._recording = False
                self._awaiting_label = True
                self._can_redo = True
                self.rec_stop.setEnabled(False)
                self._apply_record_state()
                self.record_hint.setText("已停止。这条算成功、失败，还是重录？")
                self._sync_data_dialog()
            elif kind == "success":
                send_record("success")
                self._awaiting_label = False
                self._can_redo = True
                self._apply_record_state()
                self.record_hint.setText("已标为成功。可以录下一条，或重录刚刚这一条。")
                self._sync_data_dialog()
            elif kind == "fail":
                send_record("fail")
                self._awaiting_label = False
                self._can_redo = True
                self._apply_record_state()
                self.record_hint.setText("已标为失败。可以录下一条，或重录刚刚这一条。")
                self._sync_data_dialog()
            elif kind == "redo":
                send_record("redo")
                self._undo_awaiting = self._awaiting_label
                self._awaiting_label = False
                self._recording = False
                self._can_redo = False
                self._undo_until = time.monotonic() + UNDO_SECONDS
                self._apply_record_state()
                self.record_hint.setText(DISCARD_HINT)
                self._sync_data_dialog()
            elif kind == "undo":
                if not self._undo_open():
                    self.record_hint.setText(UNDO_EXPIRED_HINT)
                    self._apply_record_state()
                    return False
                send_record("undo")
                self._awaiting_label = self._undo_awaiting
                self._can_redo = True
                self._undo_until = 0.0
                self._apply_record_state()
                self.record_hint.setText(RESTORED_HINT)
                self._sync_data_dialog()
            return True
        except OSError as error:
            if quiet:
                self.record_hint.setText(f"采集命令发送失败：{error}")
            else:
                QMessageBox.critical(self, "录制", str(error))
            return False

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._running():
            if (
                QMessageBox.question(self, "退出", "遥操作还在运行。停止并退出？")
                != QMessageBox.StandardButton.Yes
            ):
                event.ignore()
                return
            self._stop()
            QTimer.singleShot(300, self._wait_close)
            event.ignore()
            return
        if self.log_panel._dialog:
            self.log_panel._dialog.close()
        for source in list(self.log_panel.partials):
            self.log_panel.enqueue("", source, final=True)
        self.log_panel.flush()
        self.log_panel.save_settings()
        self._settings.setValue("log/splitterState", self.log_splitter.saveState())
        self._settings.setValue("log/height", self._log_expanded_height)
        event.accept()

    def _wait_close(self) -> None:
        if self._running():
            QTimer.singleShot(300, self._wait_close)
            return
        self.close()


def run() -> None:
    # Ensure script/ is on path for lerobot_cameras imports.
    script_dir = str(ROOT / "script")
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    app = QApplication(sys.argv)
    app.setApplicationName("OpenArm 遥操作")
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLE)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

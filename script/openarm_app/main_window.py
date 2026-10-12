"""Qt main window for OpenArm teleop / MoveIt / zero-g control."""

from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QTextCursor
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
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from lerobot_cameras import default_config, list_usb_cameras, load_specs, save_specs

from .data_manage_dialog import DataManageDialog
from .gravity_tune_dialog import GravityTuneDialog
from .replay_dialog import ReplayDialog
from .ker_trigger_dialog import KerTriggerDialog
from .services import (
    HTTP_PORT,
    ROOT,
    RUN,
    camera_view_open,
    configure_can,
    local_ip,
    quest_udp_host,
    send_record,
    sudo_is_cached,
)
from .theme import APP_STYLE, fit_window


class WorkerBridge(QObject):
    finished_ok = Signal()
    finished_fail = Signal(str)
    calib_done = Signal(bool, str)
    calib_log = Signal(str)
    motor_done = Signal(bool, str)
    idle = Signal(str)


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
        self._mirror_ready = False
        self._usb_by_label: dict = {}
        self._data_dialog: DataManageDialog | None = None
        self.bridge = WorkerBridge()
        self.bridge.finished_ok.connect(self._started)
        self.bridge.finished_fail.connect(self._fail)
        self.bridge.calib_done.connect(self._auto_calib_done)
        self.bridge.calib_log.connect(self._calib_log_line)
        self.bridge.motor_done.connect(self._read_motor_params_done)
        self.bridge.idle.connect(self._idle)
        self._calib_proc: subprocess.Popen[str] | None = None

        self._build()
        fit_window(
            self,
            preferred_w=1180,
            preferred_h=780,
            min_w=960,
            min_h=620,
        )
        self._apply_mode()
        self._apply_record_state()
        self._refresh_usb()
        self.batch_edit.setText(datetime.now().strftime("session_%Y%m%d_%H%M%S"))

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.setInterval(400)
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
                button = {
                    "left-short": self.rec_stop if self._recording else self.rec_start,
                    "left-long": self.rec_redo,
                    "right-short": self.rec_ok,
                    "right-long": self.rec_bad,
                }.get(action)
                if (self._mode() in ("sim", "real") and self._record_available()
                        and button is not None and button.isEnabled()
                        and not (action == "left-long" and self._recording)):
                    if action == "left-short" and not self._recording and not self.batch_edit.text().strip():
                        message = "请先在控制台填写批次名称。"
                    else:
                        kind = {"left-short": "stop" if self._recording else "start",
                                "left-long": "redo", "right-short": "success", "right-long": "fail"}[action]
                        ok = self._record(kind, quiet=True)
                        message = self.record_hint.text()
            except (ValueError, AttributeError, TypeError):
                message = "无效的采集快捷键请求。"
            reply = json.dumps({"ok": ok, "message": message}, ensure_ascii=False).encode()
            self._capture_socket.writeDatagram(reply, datagram.senderAddress(), datagram.senderPort())

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
            "Quest / KER 遥操 · MoveIt · 零重力 · 同向示教 · 重力补偿微调"
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
        root.addWidget(scroll, stretch=1)

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
        self.radio_mirror = QRadioButton("同向示教")
        self.radio_ker_sim = QRadioButton("KER 仿真")
        self.radio_ker_real = QRadioButton("KER 真机")
        for radio in (self.radio_ker_sim, self.radio_ker_real):
            radio.setToolTip("使用 USB / 串口 OpenArm KER 操作 V1 双臂及夹爪，可录制并回放。双扳机同时压下并保持 2 秒后才使能从臂。映射由 OPENARM_KER_CONFIG 指定，默认 config/ker.yaml；真机前须完成映射标定。")
        self.radio_mirror.setToolTip(
            "右臂重力补偿，可用手拖动；左臂按关节映射跟随（前后摆臂 j1、夹爪俯仰 j7 的角度和速度反号，其余同号），超出左臂行程时限位，夹爪同向跟随。\n"
            "夹爪受阻时会柔和保持，回开右夹爪可释放。\n启动后左臂会缓慢对齐，请清空双臂空间。无需 Quest，可录制双臂和相机数据。"
        )
        self.radio_sim.setChecked(True)
        for i, r in enumerate(
            (
                self.radio_sim,
                self.radio_real,
                self.radio_moveit,
                self.radio_zerog,
                self.radio_mirror,
                self.radio_ker_sim,
                self.radio_ker_real,
            )
        ):
            self.mode_group.addButton(r, i)
            mode_row.addWidget(r, i // 4, i % 4)
        self.mode_group.idClicked.connect(lambda _: self._apply_mode())
        self.quest_skeleton = QCheckBox("Quest 肘部与腕部追踪（用于仿真 / 真机遥操）")
        self.quest_skeleton.setChecked(os.environ.get("VR_TELEOP_MODE") == "skeleton_proximal")
        self._quest_skeleton_requested = self.quest_skeleton.isChecked()
        self.quest_skeleton.setToolTip("持续追踪肘和腕，肩部仅校准一次；手腕朝向及夹爪由手柄控制。短暂丢帧冻结并重新接入，持续丢失后需重新握持 Grip。请先在仿真中核对方向。")
        mode_row.addWidget(self.quest_skeleton, 2, 0, 1, 4)
        self.btn_ker_triggers = QPushButton("标定 KER 双扳机…")
        self.btn_ker_triggers.clicked.connect(self._calibrate_ker_triggers)
        mode_row.addWidget(self.btn_ker_triggers, 3, 0, 1, 4)
        left.addWidget(mode_box)

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
            "固定模板：config/inf.backup。自动微调在当前可用配置上继续优化，"
            "保留质量、负载和摩擦；手动微调可逐关节调节并热加载。"
        )
        g_hint.setObjectName("Hint")
        g_hint.setWordWrap(True)
        g_layout.addWidget(g_hint)
        g_btns = QHBoxLayout()
        self.btn_calib = QPushButton("自动微调")
        self.btn_calib.setObjectName("Primary")
        self.btn_calib.setToolTip(
            "机械臂会运动：腕→肘→肩停稳采数，在当前配置上小幅调整重力倍率/偏置。\n"
            "自动备份；用独立姿态验证，改善才保存；不重新辨识质量或摩擦。"
        )
        self.btn_tune = QPushButton("手动微调")
        self.btn_tune.setObjectName("Accent")
        self.btn_tune.setToolTip(
            "打开左右臂关节倍率、偏置、附件增益和摩擦调节窗口。\n"
            "可恢复 inf.backup 中的可调参数，热加载试调或保存；打开窗口不会运动。"
        )
        self.btn_calib.clicked.connect(self._auto_calib_gravity)
        self.btn_tune.clicked.connect(self._open_gravity_tune)
        g_btns.addWidget(self.btn_calib)
        g_btns.addWidget(self.btn_tune)
        g_btns.addStretch(1)
        g_layout.addLayout(g_btns)
        g_actions = QLabel(
            "自动微调会驱动机械臂运动；手动微调支持参数热加载。"
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
        self.rec_redo.clicked.connect(lambda: self._record("redo"))
        row2.addWidget(self.rec_ok)
        row2.addWidget(self.rec_bad)
        row2.addWidget(self.rec_redo)
        r_layout.addLayout(row2)
        manage_row = QHBoxLayout()
        self.btn_data_manage = QPushButton("数据管理…")
        self.btn_data_manage.clicked.connect(self._open_data_manage)
        manage_row.addWidget(self.btn_data_manage)
        self.btn_replay = QPushButton("动作回放…")
        self.btn_replay.setToolTip("Quest 遥操、KER 或手拖示教录制后，停止并释放，再选择片段自动回放；支持仿真、变速、暂停和停止。")
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
            ("左摇杆", "短按  开始 / 结束录制", "长按  丢弃重录"),
            ("右摇杆 · 停录后", "短按  标记成功", "长按  标记失败"),
        )):
            heading = QLabel(hand)
            heading.setObjectName("ShortcutTitle")
            shortcut_grid.addWidget(heading, 0, column)
            for row, text in enumerate((short, long), 1):
                label = QLabel(text)
                label.setWordWrap(True)
                shortcut_grid.addWidget(label, row, column)
        note = QLabel("长按 1 秒后松开 · A/B、X/Y 控制夹爪")
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
        self.rec_redo.setToolTip('放弃本次录制并重新开始。')
        self.btn_data_manage.setToolTip('浏览已录制数据，查看标签并管理或导出记录。')

        # Log
        log_box = QGroupBox("日志")
        log_layout = QVBoxLayout(log_box)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(120)
        log_layout.addWidget(self.log)
        log_toggle = QCheckBox("显示运行日志")
        log_toggle.toggled.connect(log_box.setVisible)
        right.addWidget(log_toggle)
        log_box.setVisible(False)
        right.addWidget(log_box)
        right.addStretch(1)

    def _mode(self) -> str:
        if self.radio_ker_sim.isChecked():
            return "ker_sim"
        if self.radio_ker_real.isChecked():
            return "ker_real"
        if self.radio_real.isChecked():
            return "real"
        if self.radio_moveit.isChecked():
            return "moveit"
        if self.radio_zerog.isChecked():
            return "zerog"
        if self.radio_mirror.isChecked():
            return "mirror"
        return "sim"

    def _hardware_mode(self, mode: str | None = None) -> bool:
        mode = mode or self._mode()
        if mode in ("real", "zerog", "mirror", "ker_real"):
            return True
        if mode == "moveit":
            return self.safety.isChecked()
        return False

    def _running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _record_available(self) -> bool:
        mode = self._mode()
        return (self._running() and mode not in ("moveit", "zerog")
                and (mode not in ("mirror", "ker_sim", "ker_real") or self._mirror_ready))

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
        self.quest_skeleton.setEnabled(mode in ("sim", "real") and not self._running())
        self.quest_label.setVisible(mode not in ("ker_sim", "ker_real"))
        can_ui = mode in ("real", "moveit", "zerog", "mirror", "ker_real")
        if not self._running():
            self._set_can_enabled(can_ui)
            if mode in ("sim", "ker_sim"):
                self.safety.setChecked(False)
            if mode == "moveit":
                self.status.setText(
                    "MoveIt 实机（已勾选急停）"
                    if self.safety.isChecked()
                    else "MoveIt 仿真（未勾选急停）"
                )
            elif mode == "zerog":
                self.status.setText("零重力：勾选急停后启动（含关节补偿/摩擦）")
            elif mode == "mirror":
                self.status.setText("同向示教：右手拖动、左臂同向跟随；勾选安全确认后启动")
            elif mode in ("ker_sim", "ker_real"):
                self.status.setText("KER：启动后双扳机同时压住 2 秒才开始跟随")
            elif self.status.text().startswith(("MoveIt", "零重力", "同向示教", "KER")):
                self.status.setText("未启动")
        self._apply_record_state()

    def _calibrate_ker_triggers(self) -> None:
        if self._running() or not self.btn_start.isEnabled():
            return
        try:
            KerTriggerDialog(self).exec()
        except (ValueError, OSError) as error:
            QMessageBox.warning(self, "KER 扳机标定", str(error))

    def _apply_record_state(self) -> None:
        running = self._running()
        self.btn_ker_triggers.setEnabled(not running)
        self.btn_replay.setEnabled(not running)
        for r in (
            self.radio_sim,
            self.radio_real,
            self.radio_moveit,
            self.radio_zerog,
            self.radio_mirror,
            self.radio_ker_sim,
            self.radio_ker_real,
        ):
            r.setEnabled(not running)
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)
        self.btn_calib.setEnabled(not running)
        self.btn_motor.setEnabled(not running)
        if running:
            self._set_can_enabled(False)
        elif self._mode() in ("real", "moveit", "zerog", "mirror", "ker_real"):
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
        self.rec_redo.setEnabled(teleop and self._can_redo and not self._recording)
        self.batch_edit.setEnabled(teleop and not self._awaiting_label and not self._recording)

    def _append_log(self, text: str) -> None:
        self.log.moveCursor(QTextCursor.MoveOperation.End)
        self.log.insertPlainText(text)
        self.log.moveCursor(QTextCursor.MoveOperation.End)

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
        if self._running():
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
                "mirror": "同向示教",
                "ker_real": "KER 真机",
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
        if mode == "mirror" and QMessageBox.question(
            self, "启动同向示教",
            "右臂将进入可拖动的重力补偿，左臂会从当前位置缓慢对齐右臂映射目标（前后摆臂 j1、夹爪俯仰 j7 反号，其余同号，受左臂限位约束）。\n"
            "夹爪同向跟随。请清空双臂工作区，启动后只拖动右臂。\n\n继续？",
        ) != QMessageBox.StandardButton.Yes:
            return
        password = None
        if mode in ("ker_sim", "ker_real"):
            try:
                from ker_bridge import ker_python, load_mapping
                load_mapping(Path(os.environ.get("OPENARM_KER_CONFIG", str(ROOT / "config/ker.yaml"))),
                             real=mode == "ker_real")
                subprocess.run([ker_python(), "-c", "from openarm_ker.ker_stream import KERStream; import usb.core, serial"],
                               check=True, capture_output=True, text=True, timeout=10)
            except (OSError, ValueError, ImportError, subprocess.SubprocessError) as error:
                QMessageBox.warning(self, "KER 启动检查", str(error) + "\n\nKER 依赖安装：bash scripts/install_ker.sh")
                return
        if real:
            password = self._ask_sudo()
            if password is None:
                return
        if mode == "moveit":
            self.status.setText("正在启动 MoveIt 实机…" if real else "正在启动 MoveIt 仿真…")
        elif mode == "zerog":
            self.status.setText("正在启动零重力…")
        elif mode == "mirror":
            self.status.setText("正在启动同向示教…")
        elif mode in ("ker_sim", "ker_real"):
            self.status.setText("正在启动 KER…")
        else:
            self.status.setText("正在配置并启动…")
        self.btn_start.setEnabled(False)
        self.btn_ker_triggers.setEnabled(False)
        self._quest_skeleton_requested = self.quest_skeleton.isChecked()
        self.quest_skeleton.setEnabled(False)
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
        self._mirror_ready = False
        environment = os.environ.copy()
        environment["OPENARM_RECORD_PANEL"] = "0"
        environment["OPENARM_LEFT_FOLLOWER_CAN"] = left
        environment["OPENARM_RIGHT_FOLLOWER_CAN"] = right
        environment.setdefault("OPENARM_ROOT", str(ROOT))
        environment.setdefault("OPENARM_GRAVITY_YAML", str(ROOT / "config" / "gravity_comp.yaml"))
        self.log_handle = self.log_path.open("wb", buffering=0)
        if mode == "moveit":
            use_real = self._hardware_mode(mode)
            environment["OPENARM_MOVEIT_REAL"] = "true" if use_real else "false"
            command = [
                "bash",
                str(ROOT / "scripts" / "run_moveit.sh"),
                "--real" if use_real else "--sim",
            ]
        elif mode == "zerog":
            command = ["bash", str(ROOT / "scripts" / "run_zero_g.sh")]
        elif mode == "mirror":
            command = ["bash", str(RUN), "--mirror"]
        elif mode in ("ker_sim", "ker_real"):
            command = ["bash", str(RUN), "--ker", "--real" if mode == "ker_real" else "--sim"]
        else:
            command = ["bash", str(RUN), "--real" if mode == "real" else "--sim"]
            if self._quest_skeleton_requested:
                command.append("--skeleton")
            elif environment.get("VR_TELEOP_MODE") == "skeleton_proximal":
                environment["VR_TELEOP_MODE"] = "human_mimic"
        self.proc = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def _started(self) -> None:
        mode = self._mode()
        if mode == "moveit":
            kind = "MoveIt 实机" if self.safety.isChecked() else "MoveIt 仿真"
        elif mode == "zerog":
            kind = "零重力"
        elif mode == "mirror":
            kind = "同向示教"
        elif mode in ("ker_sim", "ker_real"):
            kind = "KER 真机" if mode == "ker_real" else "KER 仿真"
        else:
            kind = {"real": "真机", "sim": "仿真"}.get(mode, "仿真")
        self.status.setText(f"{kind}编译/启动中…" if mode in ("mirror", "ker_sim", "ker_real") else f"{kind}运行中")
        if mode == "zerog":
            self.record_hint.setText("零重力已托住双臂。可用「重力」分区微调；不录遥操数据。")
        elif mode == "mirror":
            self.record_hint.setText(
                "正在准备控制器，首次编译可能需要等待；就绪后左臂会慢速对齐，对齐完成再开放录制。"
            )
        elif mode in ("ker_sim", "ker_real"):
            self.record_hint.setText("连接 KER，双扳机同时压下并保持 2 秒。确认前从臂不使能；通过后保持主臂姿态，等待慢速对齐。断连须重新启动并确认。")
        elif mode == "moveit" and self.safety.isChecked():
            self.record_hint.setText("MoveIt 实机：规划验证后可用「重力」分区热加载参数。")
        elif mode == "moveit":
            self.record_hint.setText("MoveIt 仿真只在 RViz 里动模型，不录遥操数据。")
        elif mode in ("sim", "real") and self._quest_skeleton_requested:
            self.record_hint.setText("肘腕模式：确认左右追踪有效后握持 Grip。短暂丢帧保持并自动重新接入；持续丢失后需松开并重新握持。")
        else:
            self.record_hint.setText("可以开始录。停下来之后选择成功、失败或重录。")
        self._awaiting_label = False
        self._recording = False
        self._can_redo = False
        self._apply_record_state()
        self._poll_timer.start()
        self._sync_data_dialog()

    def _fail(self, text: str) -> None:
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
            try:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=3)
            except ProcessLookupError:
                pass
            self.bridge.idle.emit("已停止")

        threading.Thread(target=work, daemon=True).start()

    def _idle(self, text: str) -> None:
        self.proc = None
        self._close_log()
        self._awaiting_label = False
        self._recording = False
        self._can_redo = False
        self.status.setText(text)
        self.record_hint.setText("启动之后才能录。停下来再标成功、失败，或重录。")
        self._poll_timer.stop()
        self._apply_mode()
        self._apply_record_state()
        self._sync_data_dialog()

    def _poll(self) -> None:
        self._refresh_log()
        proc = self.proc
        if proc is None:
            self._poll_timer.stop()
            return
        code = proc.poll()
        if code is not None:
            self._idle(f"控制器已退出，返回码 {code}")

    def _refresh_log(self) -> None:
        if not self.log_path.is_file():
            return
        text = self.log_path.read_text(encoding="utf-8", errors="replace")
        if self._mode() in ("ker_sim", "ker_real") and not self._mirror_ready and "[ker] TRACKING:" in text:
            self._mirror_ready = True
            self.status.setText("KER 跟随运行中")
            self.record_hint.setText("双臂及夹爪跟随 KER。可开始录制；停录并标注后停止控制器，在「动作回放」中重放。")
            self._apply_record_state()
            self._sync_data_dialog()
        elif self._mode() in ("ker_sim", "ker_real") and not self._mirror_ready:
            if "[ker] ARMED:" in text:
                self.status.setText("KER：双扳机确认通过，正在慢速对齐…")
            elif text.rfind("[ker] WAITING_POSE:") > text.rfind("[ker] POSE_VALID:"):
                self.status.setText("KER：准备姿态超限，从臂未使能；请查看日志中的关节及映射")
            elif "[ker] WAITING_TRIGGERS:" in text:
                self.status.setText("KER：等待双扳机同时压住 2 秒（从臂未使能）")
        elif self._mode() == "mirror" and not self._mirror_ready and "[mirror] TRACKING:" in text:
            self._mirror_ready = True
            self.status.setText("同向示教运行中")
            self.record_hint.setText("拖动右臂，左臂同向跟随（含夹爪）。可开始录制双臂与相机数据。")
            self._apply_record_state()
            self._sync_data_dialog()
        elif self._mode() == "mirror" and not self._mirror_ready and "[mirror] READY:" in text:
            self.status.setText("同向示教：左臂正在慢速对齐…")
            self.record_hint.setText("控制器已就绪，等待左臂对齐；出现 TRACKING 后开放录制。")
        tail = "\n".join(text.splitlines()[-80:])
        self.log.setPlainText(tail)
        self.log.moveCursor(QTextCursor.MoveOperation.End)

    def _close_log(self) -> None:
        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None

    def _open_gravity_tune(self) -> None:
        dlg = GravityTuneDialog(self)
        dlg.exec()

    def _auto_calib_gravity(self) -> None:
        title = "自动微调"
        if self._running():
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
        yaml_path = ROOT / "config" / "gravity_comp.yaml"
        if (
            QMessageBox.question(
                self,
                title,
                "在已有方案上自动微调（约数分钟；模板 inf.backup）：\n"
                "1. 备份并保留当前质量/质心、负载、相机和摩擦\n"
                "2. 腕→肘→肩航点：到位、停稳后再软持采数（移动时不记录）\n"
                "3. 按零重力同一 KDL 模型精修关节倍率/偏置\n"
                "4. 用未参与拟合的姿态验证，改善才写回\n\n"
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
        self.btn_motor.setEnabled(False)
        self.btn_start.setEnabled(False)
        self.btn_tune.setEnabled(False)
        self.status.setText("正在自动微调…（日志区有实时输出）")
        self._append_log("\n--- 自动微调开始 ---\n")
        threading.Thread(
            target=self._auto_calib_worker,
            args=(left, right, bitrate, dbitrate, password),
            daemon=True,
        ).start()

    def _backup_gravity_yaml(self) -> Path | None:
        yaml_path = ROOT / "config" / "gravity_comp.yaml"
        if not yaml_path.is_file():
            return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = yaml_path.with_name(f"gravity_comp.yaml.bak.{stamp}")
        backup.write_bytes(yaml_path.read_bytes())
        return backup

    def _auto_calib_worker(
        self, left: str, right: str, bitrate: str, dbitrate: str, password: str | None
    ) -> None:
        script = ROOT / "scripts" / "calibrate_gravity.py"
        chunks: list[str] = []
        try:
            backup = self._backup_gravity_yaml()
            if backup is not None:
                self.bridge.calib_log.emit(f"[calib] 已备份 {backup}\n")
            configure_can(left, right, bitrate, dbitrate, password)
            env = os.environ.copy()
            env["OPENARM_LEFT_FOLLOWER_CAN"] = left
            env["OPENARM_RIGHT_FOLLOWER_CAN"] = right
            env["OPENARM_ROOT"] = str(ROOT)
            env["OPENARM_GRAVITY_YAML"] = str(ROOT / "config" / "gravity_comp.yaml")
            env["PYTHONUNBUFFERED"] = "1"
            self._calib_proc = subprocess.Popen(
                [sys.executable, "-u", str(script), "--trajectory", "waypoints"],
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
        self._append_log(line)
        if "backend=" in line or "自动标定完成" in line or "LOG_PATH=" in line:
            self.status.setText(line.strip()[:120])

    def _auto_calib_done(self, ok: bool, output: str) -> None:
        self.btn_calib.setEnabled(True)
        self.btn_tune.setEnabled(True)
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
            "请启动「零重力」检查手感，需要时使用「手动微调」。",
        )

    def _read_motor_params(self) -> None:
        if self._running():
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
        self.btn_start.setEnabled(False)
        self.btn_calib.setEnabled(False)
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
        self.btn_motor.setEnabled(True)
        self.btn_start.setEnabled(True)
        running = self._running()
        self.btn_calib.setEnabled(not running)
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
                self._awaiting_label = False
                self._recording = False
                self._can_redo = False
                self._apply_record_state()
                self.record_hint.setText("这一条已丢弃。再次开始会用同一个序号。")
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

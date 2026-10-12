"""Qt joint fine-tune dialog (scale/bias + distal gains + friction)."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QWheelEvent
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from .gravity_knobs import (
    JOINT_NAMES,
    read_knobs,
    write_knobs,
)
from .services import ROOT, resolve_gravity_yaml, sighup_gravity_processes
from .theme import APP_STYLE, fit_window

EE_LO, EE_HI = 0.0, 1.5
CAM_LO, CAM_HI = 0.0, 1.5
FC_LO, FC_HI = 0.0, 1.20
FV_LO, FV_HI = 0.0, 0.10
SCALE_LO, SCALE_HI = 0.50, 1.50
BIAS_LO, BIAS_HI = -0.50, 0.50
MOTOR_LO, MOTOR_HI = 0.0, 0.50
CAMKG_LO, CAMKG_HI = 0.0, 0.30
PAYLOAD_LO, PAYLOAD_HI = 0.0, 0.40
GSCALE_LO, GSCALE_HI = 0.70, 1.10


class WheelSlider(QWidget):
    """Horizontal slider + spinbox; mouse wheel nudges value."""

    def __init__(
        self,
        lo: float,
        hi: float,
        value: float,
        digits: int,
        on_change,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.lo = lo
        self.hi = hi
        self.digits = digits
        self._on_change = on_change
        span = max(hi - lo, 1e-6)
        self._steps = int(round(span / (10 ** (-digits))))
        self._steps = max(self._steps, 1)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, self._steps)
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(digits)
        self.spin.setRange(lo, hi)
        self.spin.setSingleStep(10 ** (-digits))
        self.spin.setMinimumWidth(90)
        layout.addWidget(self.slider, stretch=1)
        layout.addWidget(self.spin)

        self._block = False
        self.set_value(value)
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)

    def value(self) -> float:
        return float(self.spin.value())

    def set_value(self, value: float) -> None:
        value = min(self.hi, max(self.lo, float(value)))
        self._block = True
        self.spin.setValue(value)
        pos = int(round((value - self.lo) / (self.hi - self.lo) * self._steps))
        self.slider.setValue(pos)
        self._block = False

    def _from_slider(self, pos: int) -> None:
        if self._block:
            return
        value = self.lo + (self.hi - self.lo) * (pos / self._steps)
        self._block = True
        self.spin.setValue(value)
        self._block = False
        self._on_change()

    def _from_spin(self, value: float) -> None:
        if self._block:
            return
        pos = int(round((value - self.lo) / (self.hi - self.lo) * self._steps))
        self._block = True
        self.slider.setValue(pos)
        self._block = False
        self._on_change()

    def wheelEvent(self, event: QWheelEvent) -> None:  # noqa: N802
        delta = event.angleDelta().y()
        if delta == 0:
            return
        step = 10 ** (-self.digits)
        self.set_value(self.value() + (step if delta > 0 else -step))
        self._on_change()
        event.accept()


class GravityTuneDialog(QDialog):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("手动微调")
        self.yaml_path = resolve_gravity_yaml()
        self.knobs = read_knobs(self.yaml_path)
        self._saved = self._clone(self.knobs)
        self._dirty = False
        self._suppress = False
        self._fullscreen = False
        self._side = "left"

        if parent is None:
            self.setStyleSheet(APP_STYLE)
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        head = QHBoxLayout()
        title = QLabel("手动微调")
        title.setObjectName("HeroTitle")
        head.addWidget(title)
        self.status = QLabel(f"已加载：{self.yaml_path}")
        self.status.setObjectName("Hint")
        self.status.setWordWrap(True)
        head.addWidget(self.status, stretch=1)
        tools = QHBoxLayout()
        self.side_box = QComboBox()
        self.side_box.addItem("左臂", "left")
        self.side_box.addItem("右臂", "right")
        self.side_box.setToolTip("切换左右臂的倍率、偏置和摩擦；末端/相机增益及下方全局参数左右共用。")
        tools.addWidget(self.side_box)
        self.btn_reload = QPushButton("重新加载")
        self.btn_reset = QPushButton("恢复模板")
        self.btn_trial = QPushButton("热加载试调")
        self.btn_trial.setObjectName("Accent")
        self.btn_save = QPushButton("永久写入")
        self.btn_save.setObjectName("Primary")
        self.btn_full = QPushButton("全屏")
        for b in (self.btn_reload, self.btn_reset, self.btn_trial, self.btn_save, self.btn_full):
            tools.addWidget(b)
        tools.addStretch(1)
        self.btn_reload.setToolTip('从当前配置重新读取，丢弃尚未写入的界面修改；已热加载的值仍在配置中。')
        self.btn_reset.setToolTip('读取 config/inf.backup，恢复左右臂及全局的可调参数到界面；试调或保存后生效。')
        self.btn_trial.setToolTip('将界面参数写入当前配置，并通知零重力/MoveIt 进程热加载；关闭时可选择回退。')
        self.btn_save.setToolTip('保存当前参数并热加载，作为后续启动和回退的基准；不修改 inf.backup。')
        self.btn_full.setToolTip('切换全屏显示，方便逐关节调节。')
        root.addLayout(head)
        root.addLayout(tools)
        actions_hint = QLabel(
            "重新加载：读取当前配置 ｜ 恢复模板：载入 inf.backup 的可调参数\n"
            "热加载试调：写入并试用，关闭可回退 ｜ 永久写入：保存为后续基准 ｜ 全屏：展开窗口"
        )
        actions_hint.setObjectName("Hint")
        actions_hint.setWordWrap(True)
        root.addWidget(actions_hint)

        hint = QLabel(
            "基于 inf.backup 模板逐项微调，每次小幅修改并检查手感。"
            "倍率决定重力补偿强度，偏置修正固定方向力矩；附件增益与全局参数左右共用。"
            "热加载试调也会写入当前配置；关闭时可选择保存或恢复打开窗口时的参数。"
        )
        hint.setObjectName("Hint")
        hint.setWordWrap(True)
        root.addWidget(hint)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        board = QWidget()
        grid = QGridLayout(board)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        headers = ("关节", "重力倍率", "力矩偏置", "末端增益", "相机增益", "库仑摩擦", "粘性摩擦")
        explanations = (
            "七个臂关节，夹爪参数在下方。",
            "缩放当前关节的模型重力；增大后补偿更强。",
            "固定力矩偏置，单位 Nm；正负方向与关节坐标一致。",
            "各关节末端附件重力贡献的倍率，左右共用。",
            "各关节相机重力贡献的倍率，左右共用。",
            "随运动方向改变的摩擦补偿强度，单位 Nm。",
            "随速度增加的摩擦补偿，单位 Nm·s/rad。",
        )
        for col, text in enumerate(headers):
            lab = QLabel(text)
            lab.setToolTip(explanations[col])
            lab.setStyleSheet("font-weight:700; font-size:14px; color:#334e68;")
            grid.addWidget(lab, 0, col)
        self.scale: list[WheelSlider] = []
        self.bias: list[WheelSlider] = []
        self.ee: list[WheelSlider] = []
        self.cam: list[WheelSlider] = []
        self.fc: list[WheelSlider] = []
        self.fv: list[WheelSlider] = []
        for i, name in enumerate(JOINT_NAMES):
            row = i + 1
            grid.addWidget(QLabel(name), row, 0)
            w_s = WheelSlider(SCALE_LO, SCALE_HI, 1.0, 3, self._mark_dirty)
            w_b = WheelSlider(BIAS_LO, BIAS_HI, 0.0, 3, self._mark_dirty)
            w_ee = WheelSlider(EE_LO, EE_HI, self.knobs["ee"][i], 3, self._mark_dirty)
            w_cam = WheelSlider(CAM_LO, CAM_HI, self.knobs["cam"][i], 3, self._mark_dirty)
            w_fc = WheelSlider(FC_LO, FC_HI, 0.0, 3, self._mark_dirty)
            w_fv = WheelSlider(FV_LO, FV_HI, 0.0, 4, self._mark_dirty)
            self.scale.append(w_s)
            self.bias.append(w_b)
            self.ee.append(w_ee)
            self.cam.append(w_cam)
            self.fc.append(w_fc)
            self.fv.append(w_fv)
            grid.addWidget(w_s, row, 1)
            grid.addWidget(w_b, row, 2)
            grid.addWidget(w_ee, row, 3)
            grid.addWidget(w_cam, row, 4)
            grid.addWidget(w_fc, row, 5)
            grid.addWidget(w_fv, row, 6)
        scroll.setWidget(board)
        root.addWidget(scroll, stretch=1)

        foot = QGroupBox("全局 / 夹爪 j8（左右共用）")
        form = QFormLayout(foot)
        self.g_scale = WheelSlider(
            GSCALE_LO, GSCALE_HI, self.knobs["g_scale"], 3, self._mark_dirty
        )
        self.ee_payload = WheelSlider(
            PAYLOAD_LO, PAYLOAD_HI, self.knobs["ee_payload"], 3, self._mark_dirty
        )
        self.motor = WheelSlider(MOTOR_LO, MOTOR_HI, self.knobs["motor"], 3, self._mark_dirty)
        self.cam_kg = WheelSlider(CAMKG_LO, CAMKG_HI, self.knobs["cam_kg"], 3, self._mark_dirty)
        self.gfc = WheelSlider(FC_LO, FC_HI, self.knobs["gfc"], 3, self._mark_dirty)
        self.gfv = WheelSlider(FV_LO, FV_HI, self.knobs["gfv"], 4, self._mark_dirty)
        form.addRow("g_scale / ff_gain", self.g_scale)
        form.addRow("ee_payload kg", self.ee_payload)
        form.addRow("夹爪电机 kg", self.motor)
        form.addRow("腕相机 kg", self.cam_kg)
        form.addRow("夹爪 fc", self.gfc)
        form.addRow("夹爪 fv", self.gfv)
        root.addWidget(foot)

        self._apply_side_to_widgets()

        self.side_box.currentIndexChanged.connect(self._on_side_changed)
        self.btn_reload.clicked.connect(self._reload)
        self.btn_reset.clicked.connect(self._reset)
        self.btn_trial.clicked.connect(lambda: self._write(permanent=False))
        self.btn_save.clicked.connect(self._save_permanent)
        self.btn_full.clicked.connect(self._toggle_fullscreen)

        fit_window(
            self,
            preferred_w=1280,
            preferred_h=780,
            min_w=1020,
            min_h=620,
        )

    @staticmethod
    def _clone(knobs: dict) -> dict:
        return {k: (list(v) if isinstance(v, list) else v) for k, v in knobs.items()}

    def _mark_dirty(self) -> None:
        if self._suppress:
            return
        self._dirty = True
        self.status.setText("已修改（未永久写入）· 可热加载试调，确定后永久写入")

    def _flush_side_from_widgets(self) -> None:
        """Save current side widgets into knobs before switching/writing."""
        suf = "l" if self._side == "left" else "r"
        self.knobs[f"scale_{suf}"] = [w.value() for w in self.scale]
        self.knobs[f"bias_{suf}"] = [w.value() for w in self.bias]
        self.knobs[f"fc_{suf}"] = [w.value() for w in self.fc]
        self.knobs[f"fv_{suf}"] = [w.value() for w in self.fv]
        self.knobs["ee"] = [w.value() for w in self.ee]
        self.knobs["cam"] = [w.value() for w in self.cam]
        self.knobs["g_scale"] = self.g_scale.value()
        self.knobs["ee_payload"] = self.ee_payload.value()
        self.knobs["motor"] = self.motor.value()
        self.knobs["cam_kg"] = self.cam_kg.value()
        self.knobs["gfc"] = self.gfc.value()
        self.knobs["gfv"] = self.gfv.value()

    def _apply_side_to_widgets(self) -> None:
        suf = "l" if self._side == "left" else "r"
        self._suppress = True
        try:
            for i in range(7):
                self.scale[i].set_value(self.knobs[f"scale_{suf}"][i])
                self.bias[i].set_value(self.knobs[f"bias_{suf}"][i])
                self.fc[i].set_value(self.knobs[f"fc_{suf}"][i])
                self.fv[i].set_value(self.knobs[f"fv_{suf}"][i])
                self.ee[i].set_value(self.knobs["ee"][i])
                self.cam[i].set_value(self.knobs["cam"][i])
            self.g_scale.set_value(self.knobs["g_scale"])
            self.ee_payload.set_value(self.knobs["ee_payload"])
            self.motor.set_value(self.knobs["motor"])
            self.cam_kg.set_value(self.knobs["cam_kg"])
            self.gfc.set_value(self.knobs["gfc"])
            self.gfv.set_value(self.knobs["gfv"])
        finally:
            self._suppress = False

    def _on_side_changed(self) -> None:
        self._flush_side_from_widgets()
        self._side = self.side_box.currentData()
        self._apply_side_to_widgets()
        self.status.setText(f"编辑 {self._side} · {self.yaml_path.name}")

    def _collect(self) -> dict:
        self._flush_side_from_widgets()
        return self._clone(self.knobs)

    def _reload(self) -> None:
        self.yaml_path = resolve_gravity_yaml()
        self.knobs = read_knobs(self.yaml_path)
        self._saved = self._clone(self.knobs)
        self._apply_side_to_widgets()
        self._dirty = False
        s = self.knobs["scale_l"]
        self.status.setText(
            f"已重新加载 {self.yaml_path.name} · "
            f"左腕 scale j5–j7=[{s[4]:.2f},{s[5]:.2f},{s[6]:.2f}]"
        )

    def _write(self, *, permanent: bool) -> bool:
        knobs = self._collect()
        try:
            write_knobs(self.yaml_path, knobs)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "写入失败", f"{self.yaml_path}\n\n{exc}")
            self.status.setText(f"写入失败：{exc}")
            return False
        hot, count = sighup_gravity_processes()
        if permanent:
            self._dirty = False
            self._saved = self._clone(knobs)
            tip = f"已保存到：\n{self.yaml_path}"
            tip += f"\n热加载×{count}" if hot else "\n无零重力/MoveIt 进程；下次启动生效"
            QMessageBox.information(self, "永久写入成功", tip)
            self.status.setText(f"已永久写入 {self.yaml_path.name}")
        else:
            self._dirty = True
            self.status.setText(
                f"已试调{'并热加载×' + str(count) if hot else ''} · 确定后请永久写入"
            )
        return True

    def _save_permanent(self) -> None:
        if (
            QMessageBox.question(
                self,
                "永久写入",
                f"固化到：\n{self.yaml_path}\n\n确定？",
            )
            != QMessageBox.StandardButton.Yes
        ):
            return
        self._write(permanent=True)

    def _reset(self) -> None:
        template = ROOT / "config" / "inf.backup"
        try:
            if not template.is_file():
                raise FileNotFoundError(f"找不到模板：{template}")
            knobs = read_knobs(template)
        except Exception as error:
            QMessageBox.critical(self, "恢复模板失败", str(error))
            return
        self.knobs = self._clone(knobs)
        self._apply_side_to_widgets()
        self._dirty = True
        self.status.setText("已恢复 inf.backup 的可调参数（尚未写入）；质量/质心等其他参数保留")

    def _toggle_fullscreen(self) -> None:
        self._fullscreen = not self._fullscreen
        if self._fullscreen:
            self.showFullScreen()
            self.btn_full.setText("退出全屏")
        else:
            self.showNormal()
            self.btn_full.setText("全屏")

    def closeEvent(self, event) -> None:  # noqa: N802
        if not self._dirty:
            event.accept()
            return
        answer = QMessageBox.question(
            self,
            "未永久写入",
            "参数已改但未永久写入。\n是=永久写入后关闭\n否=恢复上次永久参数并关闭\n取消=继续调",
            QMessageBox.StandardButton.Yes
            | QMessageBox.StandardButton.No
            | QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Cancel:
            event.ignore()
            return
        if answer == QMessageBox.StandardButton.Yes:
            if not self._write(permanent=True):
                event.ignore()
                return
        else:
            try:
                write_knobs(self.yaml_path, self._saved)
                sighup_gravity_processes()
            except Exception as exc:  # noqa: BLE001
                QMessageBox.critical(self, "恢复失败", str(exc))
                event.ignore()
                return
        event.accept()

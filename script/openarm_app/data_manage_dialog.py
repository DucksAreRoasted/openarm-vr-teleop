"""Standalone dialog for browsing / deleting / re-labeling teleop sessions."""

from __future__ import annotations

from PySide6.QtCore import QObject, QThread, QTimer, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDialog,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from . import dataset_manager as dm
from .services import send_record
from .theme import APP_STYLE, fit_window


class _ExportWorker(QObject):
    finished = Signal(bool, str)

    def __init__(self, batch: str) -> None:
        super().__init__()
        self.batch = batch

    def run(self) -> None:
        try:
            result = dm.export_batch(self.batch)
            text = (result.stdout or "") + (result.stderr or "")
            if result.returncode != 0:
                self.finished.emit(False, text.strip() or f"export exit {result.returncode}")
            else:
                self.finished.emit(True, text.strip() or "导出完成")
        except Exception as exc:  # noqa: BLE001
            self.finished.emit(False, str(exc))


class DataManageDialog(QDialog):
    replay_requested = Signal(str)
    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        current_batch: str = "",
        teleop_running: bool = False,
        recording_busy: bool = False,
        awaiting_label: bool = False,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("数据管理")
        self.setStyleSheet(APP_STYLE)
        self._teleop_running = teleop_running
        self._recording_busy = recording_busy
        self._awaiting_label = awaiting_label
        self._current_batch_name = current_batch.strip()
        self._export_thread: QThread | None = None
        self._batches: list[dm.BatchInfo] = []
        self._episodes: list[dm.EpisodeInfo] = []

        root = QVBoxLayout(self)
        top = QHBoxLayout()
        self.status = QLabel("")
        self.status.setObjectName("Hint")
        self.status.setWordWrap(True)
        top.addWidget(self.status, stretch=1)
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self.refresh)
        top.addWidget(self.btn_refresh)
        root.addLayout(top)

        body = QHBoxLayout()
        # Left: batches
        left = QVBoxLayout()
        left.addWidget(QLabel("批次"))
        self.batch_list = QListWidget()
        self.batch_list.currentItemChanged.connect(self._on_batch_changed)
        left.addWidget(self.batch_list, stretch=1)
        batch_btns = QHBoxLayout()
        self.btn_open = QPushButton("打开目录")
        self.btn_export = QPushButton("导出")
        self.btn_del_batch = QPushButton("删整批")
        self.btn_del_batch.setObjectName("Danger")
        self.btn_open.clicked.connect(self._open_batch)
        self.btn_export.clicked.connect(self._export_selected)
        self.btn_del_batch.clicked.connect(self._delete_batch)
        batch_btns.addWidget(self.btn_open)
        batch_btns.addWidget(self.btn_export)
        batch_btns.addWidget(self.btn_del_batch)
        left.addLayout(batch_btns)
        body.addLayout(left, stretch=2)

        # Right: episodes
        right = QVBoxLayout()
        filter_row = QHBoxLayout()
        filter_row.addWidget(QLabel("片段"))
        self.filter_box = QComboBox()
        self.filter_box.addItems(["全部", "成功", "失败", "未标注"])
        self.filter_box.currentIndexChanged.connect(self._fill_episode_table)
        filter_row.addWidget(self.filter_box)
        filter_row.addStretch(1)
        right.addLayout(filter_row)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["序号", "状态", "时长", "相机", "导出"])
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self._update_action_enabled)
        right.addWidget(self.table, stretch=1)
        ep_btns = QHBoxLayout()
        self.btn_ok = QPushButton("改成功")
        self.btn_fail = QPushButton("改失败")
        self.btn_rerecord = QPushButton("重采此条")
        self.btn_replay = QPushButton("回放此条")
        self.btn_replay.setToolTip("载入选中片段的双臂轨迹，停止其他控制器后可仿真或真机自动回放。")
        self.btn_del_ep = QPushButton("删片段")
        self.btn_del_ep.setObjectName("Danger")
        self.btn_ok.clicked.connect(lambda: self._set_outcome("success"))
        self.btn_fail.clicked.connect(lambda: self._set_outcome("failure"))
        self.btn_rerecord.clicked.connect(self._rerecord)
        self.btn_replay.clicked.connect(self._replay)
        self.btn_del_ep.clicked.connect(self._delete_episode)
        ep_btns.addWidget(self.btn_ok)
        ep_btns.addWidget(self.btn_fail)
        ep_btns.addWidget(self.btn_rerecord)
        ep_btns.addWidget(self.btn_replay)
        ep_btns.addWidget(self.btn_del_ep)
        right.addLayout(ep_btns)
        body.addLayout(right, stretch=3)
        root.addLayout(body, stretch=1)

        close_row = QHBoxLayout()
        close_row.addStretch(1)
        close_btn = QPushButton("关闭")
        close_btn.clicked.connect(self.accept)
        close_row.addWidget(close_btn)
        root.addLayout(close_row)

        fit_window(self, preferred_w=900, preferred_h=560, min_w=720, min_h=420)
        self.refresh()
        if self._current_batch_name:
            self._select_batch(self._current_batch_name)

    def set_teleop_state(
        self,
        *,
        teleop_running: bool,
        recording_busy: bool = False,
        awaiting_label: bool = False,
        current_batch: str = "",
    ) -> None:
        self._teleop_running = teleop_running
        self._recording_busy = recording_busy
        self._awaiting_label = awaiting_label
        if current_batch:
            self._current_batch_name = current_batch.strip()
        self._update_action_enabled()

    def refresh(self) -> None:
        selected = self._selected_batch_name()
        self._batches = dm.list_batches()
        self.batch_list.blockSignals(True)
        self.batch_list.clear()
        for batch in self._batches:
            export = []
            if batch.has_openarm:
                export.append("OA")
            if batch.has_lerobot:
                export.append("LR")
            export_s = ",".join(export) if export else "—"
            text = (
                f"{batch.name}  ·  {batch.episodes}条 "
                f"(✓{batch.success} ✗{batch.failure} ?{batch.unlabeled})  "
                f"{dm.format_size(batch.size_bytes)}  [{export_s}]"
            )
            item = QListWidgetItem(text)
            item.setData(Qt.ItemDataRole.UserRole, batch.name)
            self.batch_list.addItem(item)
        self.batch_list.blockSignals(False)
        target = selected or self._current_batch_name
        if target:
            self._select_batch(target)
        elif self.batch_list.count():
            self.batch_list.setCurrentRow(0)
        else:
            self._episodes = []
            self._fill_episode_table()
        self.status.setText(
            f"{dm.SESSIONS_ROOT}  ·  {len(self._batches)} 个批次"
            + ("  ·  遥操运行中" if self._teleop_running else "")
        )
        self._update_action_enabled()

    def _select_batch(self, name: str) -> None:
        for i in range(self.batch_list.count()):
            item = self.batch_list.item(i)
            if item and item.data(Qt.ItemDataRole.UserRole) == name:
                self.batch_list.setCurrentRow(i)
                return

    def _selected_batch_name(self) -> str | None:
        item = self.batch_list.currentItem()
        if item is None:
            return None
        return str(item.data(Qt.ItemDataRole.UserRole))

    def _selected_episode(self) -> dm.EpisodeInfo | None:
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            return None
        row = rows[0].row()
        item = self.table.item(row, 0)
        if item is None:
            return None
        eid = item.data(Qt.ItemDataRole.UserRole)
        for ep in self._episodes:
            if ep.id == eid:
                return ep
        return None

    def _on_batch_changed(self, *_args) -> None:
        name = self._selected_batch_name()
        if not name:
            self._episodes = []
            self._fill_episode_table()
            return
        self._episodes = dm.list_episodes(name)
        self._fill_episode_table()
        self._update_action_enabled()

    def _fill_episode_table(self) -> None:
        filt = self.filter_box.currentText()
        mapping = {"成功": "success", "失败": "failure", "未标注": "unlabeled"}
        want = mapping.get(filt)
        rows = [e for e in self._episodes if want is None or e.outcome == want]
        self.table.setRowCount(len(rows))
        labels = {"success": "成功", "failure": "失败", "unlabeled": "未标注"}
        for i, ep in enumerate(rows):
            cells = [
                ep.id,
                labels.get(ep.outcome, ep.outcome),
                f"{ep.duration_s:.1f}s / {ep.frames}",
                f"{ep.cameras_ok}/{len(dm.CAMERA_NAMES)}",
                "是" if ep.in_export else "否",
            ]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col == 0:
                    item.setData(Qt.ItemDataRole.UserRole, ep.id)
                self.table.setItem(i, col, item)
        self._update_action_enabled()

    def _busy_exporting(self) -> bool:
        return self._export_thread is not None and self._export_thread.isRunning()

    def _guard_active_batch(self, batch: str) -> bool:
        """Block mutating the live recording take."""
        if not self._teleop_running:
            return True
        if batch != self._current_batch_name:
            return True
        if self._recording_busy or self._awaiting_label:
            QMessageBox.warning(
                self,
                "数据管理",
                "当前批次正在录制或等待标注，请先在「数据采集」里停录并处理完再改。",
            )
            return False
        return True

    def _update_action_enabled(self) -> None:
        batch = self._selected_batch_name()
        ep = self._selected_episode()
        exporting = self._busy_exporting()
        live_lock = (
            self._teleop_running
            and batch == self._current_batch_name
            and (self._recording_busy or self._awaiting_label)
        )
        self.btn_export.setEnabled(bool(batch) and not exporting)
        self.btn_del_batch.setEnabled(bool(batch) and not exporting and not live_lock)
        self.btn_open.setEnabled(bool(batch) or dm.SESSIONS_ROOT.is_dir())
        can_ep = bool(batch and ep) and not exporting and not live_lock
        self.btn_ok.setEnabled(can_ep)
        self.btn_fail.setEnabled(can_ep)
        self.btn_del_ep.setEnabled(can_ep)
        # Reuse segment number only while teleop is running (UDP rerecord).
        self.btn_rerecord.setEnabled(can_ep and self._teleop_running)
        self.btn_replay.setEnabled(can_ep and not self._teleop_running)

    def _replay(self) -> None:
        episode = self._selected_episode()
        if episode is not None:
            self.replay_requested.emit(str(episode.path))

    def _open_batch(self) -> None:
        name = self._selected_batch_name()
        path = dm.batch_path(name) if name else dm.SESSIONS_ROOT
        dm.open_in_file_manager(path)

    def _delete_batch(self) -> None:
        name = self._selected_batch_name()
        if not name:
            return
        if not self._guard_active_batch(name):
            return
        reply = QMessageBox.question(
            self,
            "删整批",
            f"删除整个批次「{name}」及其导出？此操作不可恢复。",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            dm.delete_batch(name)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "删整批", str(exc))
            return
        self.refresh()

    def _delete_episode(self) -> None:
        name = self._selected_batch_name()
        ep = self._selected_episode()
        if not name or ep is None:
            return
        if not self._guard_active_batch(name):
            return
        reply = QMessageBox.question(
            self,
            "删片段",
            f"删除 {name}/{ep.id} 并重导该批次？",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            dm.delete_episode(name, ep.id, export=False)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "删片段", str(exc))
            return
        self._run_export(name)

    def _set_outcome(self, outcome: str) -> None:
        name = self._selected_batch_name()
        ep = self._selected_episode()
        if not name or ep is None:
            return
        if not self._guard_active_batch(name):
            return
        try:
            dm.set_outcome(name, ep.id, outcome, export=False)  # type: ignore[arg-type]
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "改标签", str(exc))
            return
        self._run_export(name)

    def _rerecord(self) -> None:
        name = self._selected_batch_name()
        ep = self._selected_episode()
        if not name or ep is None:
            return
        if not self._teleop_running:
            QMessageBox.information(self, "重采", "请先启动遥操，再重采以复用原序号。")
            return
        if name != self._current_batch_name:
            QMessageBox.warning(
                self,
                "重采",
                f"当前录制批次是「{self._current_batch_name or '（未定）'}」，"
                f"与所选「{name}」不一致。请在数据采集里把批次改成该名称后再重采。",
            )
            return
        if self._recording_busy:
            QMessageBox.warning(self, "重采", "请先停止当前录制。")
            return
        if self._awaiting_label:
            QMessageBox.warning(self, "重采", "请先标注或丢弃当前待标片段。")
            return
        reply = QMessageBox.question(
            self,
            "重采此条",
            f"删除 {name}/{ep.id}，下次「开始录制」将复用序号 {ep.id}。继续？",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            # batch,NN so C++ can ensure_batch_dir even before first start this session
            send_record(f"rerecord,{name},{ep.id}")
        except OSError as exc:
            QMessageBox.critical(self, "重采", str(exc))
            return
        self.status.setText(f"已请求重采 {ep.id}；请回到数据采集点「开始录制」。")
        QTimer.singleShot(400, self.refresh)

    def _export_selected(self) -> None:
        name = self._selected_batch_name()
        if not name:
            return
        self._run_export(name)

    def _run_export(self, batch: str) -> None:
        if self._busy_exporting():
            return
        self.status.setText(f"正在导出 {batch} …")
        self._update_action_enabled()
        thread = QThread(self)
        worker = _ExportWorker(batch)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)

        def _done(ok: bool, message: str) -> None:
            self.status.setText(message.splitlines()[-1] if message else ("完成" if ok else "失败"))
            if not ok:
                QMessageBox.warning(self, "导出", message or "导出失败")
            thread.quit()
            self.refresh()

        worker.finished.connect(_done)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)

        def _clear() -> None:
            self._export_thread = None
            self._update_action_enabled()

        thread.finished.connect(_clear)
        self._export_thread = thread
        thread.start()
        self._update_action_enabled()

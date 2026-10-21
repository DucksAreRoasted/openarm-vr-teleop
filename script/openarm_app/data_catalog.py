"""Session browser used by the QML data page.

The widget dialog stays for its own tests. This catalog talks to the same
filesystem helpers and keeps the same guards around the live recording batch.
"""

from __future__ import annotations

from PySide6.QtCore import (
    Property,
    QAbstractListModel,
    QModelIndex,
    QObject,
    QThread,
    Qt,
    QTimer,
    Signal,
    Slot,
)
from PySide6.QtWidgets import QMessageBox

from . import dataset_manager as dm
from .services import send_record
from .trajectory_smooth_dialog import TrajectorySmoothDialog


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


class BatchModel(QAbstractListModel):
    NameRole = Qt.ItemDataRole.UserRole + 1
    SummaryRole = Qt.ItemDataRole.UserRole + 2

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._rows: list[tuple[str, str]] = []

    def roleNames(self):  # noqa: N802
        return {self.NameRole: b"batchName", self.SummaryRole: b"summary"}

    def rowCount(self, parent=QModelIndex()):  # noqa: N802
        if parent.isValid():
            return 0
        return len(self._rows)

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):  # noqa: N802
        if not index.isValid() or not (0 <= index.row() < len(self._rows)):
            return None
        name, summary = self._rows[index.row()]
        if role == self.NameRole:
            return name
        if role == self.SummaryRole:
            return summary
        return None

    def replace(self, rows: list[tuple[str, str]]) -> None:
        self.beginResetModel()
        self._rows = rows
        self.endResetModel()


class EpisodeModel(QAbstractListModel):
    IdRole = Qt.ItemDataRole.UserRole + 1
    OutcomeRole = Qt.ItemDataRole.UserRole + 2
    DetailRole = Qt.ItemDataRole.UserRole + 3

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._rows: list[tuple[str, str, str]] = []

    def roleNames(self):  # noqa: N802
        return {
            self.IdRole: b"episodeId",
            self.OutcomeRole: b"outcomeLabel",
            self.DetailRole: b"detail",
        }

    def rowCount(self, parent=QModelIndex()):  # noqa: N802
        if parent.isValid():
            return 0
        return len(self._rows)

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):  # noqa: N802
        if not index.isValid() or not (0 <= index.row() < len(self._rows)):
            return None
        episode_id, outcome, detail = self._rows[index.row()]
        if role == self.IdRole:
            return episode_id
        if role == self.OutcomeRole:
            return outcome
        if role == self.DetailRole:
            return detail
        return None

    def replace(self, rows: list[tuple[str, str, str]]) -> None:
        self.beginResetModel()
        self._rows = rows
        self.endResetModel()


class DataCatalog(QObject):
    statusChanged = Signal()
    selectionChanged = Signal()
    actionsChanged = Signal()
    filterChanged = Signal()

    def __init__(self, console) -> None:
        super().__init__(console)
        self._console = console
        self.batch_model = BatchModel(self)
        self.episode_model = EpisodeModel(self)
        self._episodes: list[dm.EpisodeInfo] = []
        self._selected_batch = ""
        self._selected_episode = ""
        self._filter = "全部"
        self._status = "打开本页后加载录制批次"
        self._loaded = False
        self._export_thread: QThread | None = None
        console.uiChanged.connect(self.actionsChanged.emit)

    @Property(QObject, constant=True)
    def batches(self):
        return self.batch_model

    @Property(QObject, constant=True)
    def episodes(self):
        return self.episode_model

    @Property(str, notify=statusChanged)
    def statusText(self) -> str:
        return self._status

    @Property(str, notify=selectionChanged)
    def selectedBatch(self) -> str:
        return self._selected_batch

    @Property(str, notify=selectionChanged)
    def selectedEpisode(self) -> str:
        return self._selected_episode

    @Property(str, notify=filterChanged)
    def episodeFilter(self) -> str:
        return self._filter

    def _set_status(self, text: str) -> None:
        if text != self._status:
            self._status = text
            self.statusChanged.emit()

    def _live_lock(self, batch: str | None = None) -> bool:
        console = self._console
        name = self._selected_batch if batch is None else batch
        return bool(
            console._record_available()
            and name == console._batch_name.strip()
            and (console._recording or console._awaiting_label)
        )

    def _selected_info(self) -> dm.EpisodeInfo | None:
        for episode in self._episodes:
            if episode.id == self._selected_episode:
                return episode
        return None

    @Property(bool, notify=actionsChanged)
    def canExport(self) -> bool:
        return bool(self._selected_batch) and not self._busy()

    @Property(bool, notify=actionsChanged)
    def canDeleteBatch(self) -> bool:
        return bool(self._selected_batch) and not self._busy() and not self._live_lock()

    @Property(bool, notify=actionsChanged)
    def canOpen(self) -> bool:
        return bool(self._selected_batch) or dm.SESSIONS_ROOT.is_dir()

    @Property(bool, notify=actionsChanged)
    def canEditEpisode(self) -> bool:
        return bool(self._selected_batch and self._selected_info()) and not self._busy() and not self._live_lock()

    @Property(bool, notify=actionsChanged)
    def canRerecord(self) -> bool:
        return self.canEditEpisode and self._console._record_available()

    @Property(bool, notify=actionsChanged)
    def canReplay(self) -> bool:
        return self.canEditEpisode and not self._console._record_available()

    def _busy(self) -> bool:
        return self._export_thread is not None and self._export_thread.isRunning()

    def _guard_active_batch(self, batch: str) -> bool:
        if not self._live_lock(batch):
            return True
        QMessageBox.warning(
            None,
            "数据管理",
            "当前批次正在录制或等待标注，请先在「采集」里停录并处理完再改。",
        )
        return False

    @Slot()
    def refresh(self) -> None:
        self._loaded = True
        selected = self._selected_batch
        batches = dm.list_batches()
        rows = []
        for batch in batches:
            export = []
            if batch.has_openarm:
                export.append("OA")
            if batch.has_lerobot:
                export.append("LR")
            export_s = ",".join(export) if export else "—"
            summary = (
                f"{batch.episodes} 条  ·  成功 {batch.success}  ·  失败 {batch.failure}"
                f"  ·  未标注 {batch.unlabeled}  ·  {dm.format_size(batch.size_bytes)}  ·  {export_s}"
            )
            rows.append((batch.name, summary))
        self.batch_model.replace(rows)
        names = [name for name, _summary in rows]
        if selected not in names:
            selected = self._console._batch_name.strip() if self._console._batch_name.strip() in names else ""
        if not selected and names:
            selected = names[0]
        self._load_batch(selected)
        running = "  ·  遥操运行中" if self._console._record_available() else ""
        self._set_status(f"{dm.SESSIONS_ROOT}  ·  {len(batches)} 个批次{running}")
        self.actionsChanged.emit()

    def _load_batch(self, name: str) -> None:
        if name != self._selected_batch or not name:
            self._selected_batch = name
            self._selected_episode = ""
            self.selectionChanged.emit()
        self._episodes = dm.list_episodes(name) if name else []
        self._fill_episodes()

    def _fill_episodes(self) -> None:
        mapping = {"成功": "success", "失败": "failure", "未标注": "unlabeled"}
        want = mapping.get(self._filter)
        labels = {"success": "成功", "failure": "失败", "unlabeled": "未标注"}
        rows = []
        visible = []
        for episode in self._episodes:
            if want is not None and episode.outcome != want:
                continue
            visible.append(episode.id)
            detail = (
                f"{episode.duration_s:.1f}s / {episode.frames} 帧"
                f"  ·  相机 {episode.cameras_ok}/{len(dm.CAMERA_NAMES)}"
                f"  ·  导出 {'是' if episode.in_export else '否'}"
            )
            rows.append((episode.id, labels.get(episode.outcome, episode.outcome), detail))
        self.episode_model.replace(rows)
        if self._selected_episode not in visible:
            self._selected_episode = visible[0] if visible else ""
            self.selectionChanged.emit()
        self.actionsChanged.emit()

    @Slot(str)
    def selectBatch(self, name: str) -> None:
        if name == self._selected_batch:
            return
        self._load_batch(name)

    @Slot(str)
    def selectEpisode(self, episode_id: str) -> None:
        if episode_id == self._selected_episode:
            return
        self._selected_episode = episode_id
        self.selectionChanged.emit()
        self.actionsChanged.emit()

    @Slot(str)
    def setEpisodeFilter(self, text: str) -> None:
        if text == self._filter:
            return
        self._filter = text
        self.filterChanged.emit()
        self._fill_episodes()

    @Slot()
    def openFolder(self) -> None:
        name = self._selected_batch
        path = dm.batch_path(name) if name else dm.SESSIONS_ROOT
        dm.open_in_file_manager(path)

    @Slot()
    def exportSelected(self) -> None:
        if self._selected_batch:
            self._run_export(self._selected_batch)

    @Slot()
    def deleteBatch(self) -> None:
        name = self._selected_batch
        if not name or not self._guard_active_batch(name):
            return
        reply = QMessageBox.question(
            None,
            "删整批",
            f"删除整个批次「{name}」及其导出？此操作不可恢复。",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            dm.delete_batch(name)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(None, "删整批", str(exc))
            return
        self._selected_batch = ""
        self.refresh()

    @Slot()
    def deleteEpisode(self) -> None:
        name = self._selected_batch
        episode = self._selected_info()
        if not name or episode is None or not self._guard_active_batch(name):
            return
        reply = QMessageBox.question(None, "删片段", f"删除 {name}/{episode.id} 并重导该批次？")
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            dm.delete_episode(name, episode.id, export=False)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(None, "删片段", str(exc))
            return
        self._run_export(name)

    @Slot(str)
    def setOutcome(self, outcome: str) -> None:
        name = self._selected_batch
        episode = self._selected_info()
        if not name or episode is None or outcome not in ("success", "failure"):
            return
        if not self._guard_active_batch(name):
            return
        try:
            dm.set_outcome(name, episode.id, outcome, export=False)  # type: ignore[arg-type]
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(None, "改标签", str(exc))
            return
        self._run_export(name)

    @Slot()
    def rerecord(self) -> None:
        name = self._selected_batch
        episode = self._selected_info()
        console = self._console
        if not name or episode is None:
            return
        if not console._record_available():
            QMessageBox.information(None, "重采", "请先启动遥操，再重采以复用原序号。")
            return
        if name != console._batch_name.strip():
            QMessageBox.warning(
                None,
                "重采",
                f"当前录制批次是「{console._batch_name.strip() or '（未定）'}」，"
                f"与所选「{name}」不一致。请在采集页把批次改成该名称后再重采。",
            )
            return
        if console._recording:
            QMessageBox.warning(None, "重采", "请先停止当前录制。")
            return
        if console._awaiting_label:
            QMessageBox.warning(None, "重采", "请先标注或丢弃当前待标片段。")
            return
        reply = QMessageBox.question(
            None,
            "重采此条",
            f"删除 {name}/{episode.id}，下次「开始录制」将复用序号 {episode.id}。继续？",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            send_record(f"rerecord,{name},{episode.id}")
        except OSError as exc:
            QMessageBox.critical(None, "重采", str(exc))
            return
        self._set_status(f"已请求重采 {episode.id}；请回到采集页点「开始录制」。")
        QTimer.singleShot(400, self.refresh)

    @Slot()
    def smoothSelected(self) -> None:
        episode = self._selected_info()
        if episode is None or not self.canReplay:
            return
        dialog = TrajectorySmoothDialog(episode.path, None)
        dialog.replay_requested.connect(self._console._open_replay)
        dialog.exec()

    @Slot()
    def replaySelected(self) -> None:
        episode = self._selected_info()
        if episode is not None:
            self._console._open_replay(str(episode.path))

    def _run_export(self, batch: str) -> None:
        if self._busy():
            return
        self._set_status(f"正在导出 {batch} …")
        self.actionsChanged.emit()
        thread = QThread(self)
        worker = _ExportWorker(batch)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)

        def _done(ok: bool, message: str) -> None:
            self._set_status(message.splitlines()[-1] if message else ("完成" if ok else "失败"))
            if not ok:
                QMessageBox.warning(None, "导出", message or "导出失败")
            thread.quit()
            self.refresh()

        worker.finished.connect(_done)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)

        def _clear() -> None:
            self._export_thread = None
            self.actionsChanged.emit()

        thread.finished.connect(_clear)
        self._export_thread = thread
        thread.start()
        self.actionsChanged.emit()

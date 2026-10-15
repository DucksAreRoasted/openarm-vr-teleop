#!/usr/bin/env python3
"""Record the three Gemini 335 color streams for one teleop episode.

Writes ``<out>/<name>/NNNNNN.jpg`` plus ``times.csv`` (index, unix seconds).
The LeRobot exporter aligns these frames to the joint log. If the Orbbec SDK
or the cameras are absent, the process exits 0 so joint recording still works.
"""

from __future__ import annotations

import argparse
import io
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

WIDTH = 640
HEIGHT = 480
FPS = 30
NAMES = ("head", "wrist_left", "wrist_right")
VIEW_PORT = 47932
VIEW_TITLES = {
    "head": "头部相机",
    "wrist_left": "左腕相机",
    "wrist_right": "右腕相机",
}
ENV_BY_NAME = {
    "head": "OPENARM_CAMERA_HEAD",
    "wrist_left": "OPENARM_CAMERA_WRIST_LEFT",
    "wrist_right": "OPENARM_CAMERA_WRIST_RIGHT",
}


ORBBEC_VENDOR = "2bc5"


@dataclass
class CameraSpec:
    name: str
    serial: str = ""
    usb: str = ""


@dataclass
class UsbCamera:
    port: str
    product: str = ""
    serial: str = ""

    def label(self) -> str:
        parts = [self.port]
        if self.product:
            parts.append(self.product)
        if self.serial:
            parts.append(self.serial)
        return " · ".join(parts)


def default_config() -> Path:
    here = Path(__file__).resolve().parents[1] / "config" / "cameras.yaml"
    override = os.environ.get("OPENARM_CAMERAS_YAML", "").strip()
    return Path(override) if override else here


def load_specs(path: Path) -> list[CameraSpec]:
    specs = [CameraSpec(name) for name in NAMES]
    if path.is_file():
        current: CameraSpec | None = None
        found: list[CameraSpec] = []
        for raw in path.read_text(encoding="utf-8").splitlines():
            line = raw.split("#", 1)[0].rstrip()
            stripped = line.strip()
            if stripped.startswith("- name:"):
                current = CameraSpec(stripped.split(":", 1)[1].strip())
                found.append(current)
            elif stripped.startswith("serial:") and current is not None:
                current.serial = stripped.split(":", 1)[1].strip().strip("\"'")
            elif stripped.startswith("usb:") and current is not None:
                current.usb = stripped.split(":", 1)[1].strip().strip("\"'")
        if found:
            specs = found
    for spec in specs:
        env = os.environ.get(ENV_BY_NAME.get(spec.name, ""), "").strip()
        if env:
            spec.serial = env
    return specs


def _read_sysfs(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def list_usb_cameras(root: Path | None = None) -> list[UsbCamera]:
    """Orbbec cameras currently on the USB bus, keyed by port path (for example 2-1.4)."""
    base = root or Path("/sys/bus/usb/devices")
    found: list[UsbCamera] = []
    if not base.is_dir():
        return found
    for node in sorted(base.iterdir()):
        name = node.name
        if not node.is_dir() or ":" in name or name.startswith("usb"):
            continue
        if _read_sysfs(node / "idVendor").lower() != ORBBEC_VENDOR:
            continue
        found.append(
            UsbCamera(
                port=name,
                product=_read_sysfs(node / "product"),
                serial=_read_sysfs(node / "serial"),
            )
        )
    return found


def apply_usb_ports(specs: list[CameraSpec], root: Path | None = None) -> list[CameraSpec]:
    """If a role has a USB port, use the camera plugged into that port now."""
    by_port = {item.port: item for item in list_usb_cameras(root)}
    for spec in specs:
        plugged = by_port.get(spec.usb)
        if plugged is not None and plugged.serial:
            spec.serial = plugged.serial
    return specs


def save_specs(path: Path, chosen: dict[str, UsbCamera | None]) -> None:
    lines = [
        "# Gemini 335 cameras. usb is the sysfs port (for example 2-1.4).",
        "# The teleop window fills this by reading the USB bus.",
        "cameras:",
    ]
    for name in NAMES:
        item = chosen.get(name)
        lines.append(f"  - name: {name}")
        lines.append(f"    usb: \"{item.port if item else ''}\"")
        lines.append(f"    serial: \"{item.serial if item else ''}\"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _norm(serial: str) -> str:
    return serial.strip().upper()


def _ffmpeg_bin() -> str | None:
    found = shutil.which("ffmpeg")
    if found:
        return found
    roots = [Path.home() / "miniconda3" / "envs", Path.home() / "anaconda3" / "envs"]
    preferred = []
    for root in roots:
        preferred.extend(sorted(root.glob("lerobot/bin/ffmpeg")))
        preferred.extend(sorted(root.glob("*/bin/ffmpeg")))
    for path in preferred:
        if path.is_file():
            return str(path)
    return None


def color_device_for_port(port: str) -> str | None:
    """The Gemini color node on this USB port. Depth and IR nodes are ignored."""
    if not port:
        return None
    base = Path("/sys/class/video4linux")
    if not base.is_dir():
        return None
    best = None
    best_score = 0
    for node in sorted(base.glob("video*")):
        try:
            iface = (node / "device").resolve().name
        except OSError:
            continue
        if not iface.startswith(port + ":"):
            continue
        device = f"/dev/{node.name}"
        try:
            listed = subprocess.run(
                ["v4l2-ctl", "-d", device, "--list-formats"],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            ).stdout
        except (OSError, subprocess.TimeoutExpired):
            continue
        score = 2 if ("MJPG" in listed or "YUYV" in listed) else 0
        if score > best_score:
            best_score = score
            best = device
    return best


def port_for_serial(serial: str) -> str:
    wanted = _norm(serial)
    if not wanted:
        return ""
    for item in list_usb_cameras():
        if _norm(item.serial) == wanted:
            return item.port
    return ""


def list_serials(sdk) -> list[str]:
    context = sdk.Context()
    devices = context.query_devices()
    found = []
    try:
        count = int(devices.get_count())
    except Exception:
        count = 0
    for index in range(count):
        try:
            serial = devices.get_device_serial_number_by_index(index)
        except Exception:
            serial = ""
        if serial:
            found.append(str(serial).strip())
    return found


def bind_serials(specs: list[CameraSpec], connected: list[str]) -> list[CameraSpec]:
    used = {_norm(spec.serial) for spec in specs if spec.serial}
    free = [serial for serial in connected if _norm(serial) not in used]
    for spec in specs:
        if spec.serial:
            continue
        if not free:
            break
        spec.serial = free.pop(0)
    return specs


def resize_rgb(image: np.ndarray, width: int, height: int) -> np.ndarray:
    if image.shape[1] == width and image.shape[0] == height:
        return np.ascontiguousarray(image)
    ys = np.linspace(0, image.shape[0] - 1, height).astype(np.int32)
    xs = np.linspace(0, image.shape[1] - 1, width).astype(np.int32)
    return np.ascontiguousarray(image[ys][:, xs])


def decode_color(frame) -> np.ndarray:
    width, height = int(frame.get_width()), int(frame.get_height())
    fmt = str(getattr(frame.get_format(), "name", frame.get_format())).upper()
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)
    if "MJPG" in fmt or "JPEG" in fmt:
        import cv2

        decoded = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if decoded is None:
            raise RuntimeError("MJPG decode failed")
        return decoded[:, :, ::-1]
    if "RGB" in fmt and "BGR" not in fmt and data.size == height * width * 3:
        return data.reshape((height, width, 3))
    if data.size == height * width * 3:
        return data.reshape((height, width, 3))[:, :, ::-1]
    raise RuntimeError(f"unsupported color format {fmt}")


def save_jpeg(path: Path, rgb: np.ndarray) -> None:
    try:
        import cv2

        ok = cv2.imwrite(str(path), rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            return
    except Exception:
        pass
    from PIL import Image

    Image.fromarray(rgb, mode="RGB").save(path, quality=80)


def open_color_pipeline(sdk, serial: str):
    from pyorbbecsdk import Config, OBSensorType

    context = sdk.Context()
    devices = context.query_devices()
    wanted = _norm(serial)
    device = None
    try:
        count = int(devices.get_count())
    except Exception:
        count = 0
    for index in range(count):
        try:
            listed = devices.get_device_serial_number_by_index(index)
        except Exception:
            listed = ""
        if _norm(str(listed or "")) != wanted:
            continue
        device = devices.get_device_by_index(index)
        break
    if device is None:
        raise RuntimeError(f"Orbbec serial {serial!r} not found among {count} device(s)")
    pipeline = sdk.Pipeline(device)
    config = Config()
    config.enable_stream(
        pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
    )
    pipeline.start(config)
    # Keep the context alive for the life of the pipeline.
    pipeline._openarm_context = context
    return pipeline


def capture_one(spec: CameraSpec, out_dir: Path, stop: threading.Event, sdk) -> None:
    folder = out_dir / spec.name
    folder.mkdir(parents=True, exist_ok=True)
    times_path = folder / "times.csv"
    pipeline = None
    try:
        pipeline = open_color_pipeline(sdk, spec.serial)
        print(f"[lerobot-cam] {spec.name} serial={spec.serial}", flush=True)
        period = 1.0 / FPS
        next_t = time.monotonic()
        index = 0
        with times_path.open("w", encoding="utf-8") as times:
            times.write("index,unix\n")
            while not stop.is_set():
                frames = pipeline.wait_for_frames(200)
                if frames is None:
                    continue
                color = frames.get_color_frame()
                if not color:
                    continue
                now = time.monotonic()
                if now < next_t:
                    continue
                next_t = now + period
                rgb = resize_rgb(decode_color(color), WIDTH, HEIGHT)
                path = folder / f"{index:06d}.jpg"
                save_jpeg(path, rgb)
                times.write(f"{index},{time.time():.6f}\n")
                times.flush()
                index += 1
    except Exception as exc:
        print(f"[lerobot-cam] {spec.name} stopped: {exc}", file=sys.stderr, flush=True)
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass


def record(out_dir: Path) -> int:
    try:
        import pyorbbecsdk as sdk
    except ImportError:
        print("[lerobot-cam] pyorbbecsdk is not installed; cameras skipped", file=sys.stderr)
        return 0
    specs = bind_serials(apply_usb_ports(load_specs(default_config())), list_serials(sdk))
    armed = [spec for spec in specs if spec.serial]
    if not armed:
        print("[lerobot-cam] no Gemini 335 serials bound; cameras skipped", file=sys.stderr)
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()

    def _stop(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    threads = [
        threading.Thread(target=capture_one, args=(spec, out_dir, stop, sdk), daemon=True)
        for spec in armed
    ]
    for thread in threads:
        thread.start()
    while not stop.is_set():
        time.sleep(0.1)
    for thread in threads:
        thread.join(timeout=2.0)
    return 0


class ViewHub:
    """Latest color frame per camera, plus an optional episode directory."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.frames: dict[str, np.ndarray] = {}
        self.status: dict[str, str] = {}
        self.updated: dict[str, float] = {}
        self._root: Path | None = None
        self._writers: dict[str, list] = {}

    def set_status(self, name: str, text: str) -> None:
        with self._lock:
            self.status[name] = text

    def latest(self, name: str) -> tuple[np.ndarray | None, str]:
        with self._lock:
            frame = self.frames.get(name)
            if time.monotonic() - self.updated.get(name, 0) > 1:
                return None, self.status.get(name, "") or "相机画面已中断"
            return (None if frame is None else frame.copy(), self.status.get(name, ""))

    def begin(self, root: Path) -> None:
        root.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self._close_writers()
            self._root = root

    def end(self) -> None:
        with self._lock:
            self._close_writers()
            self._root = None

    def push(self, name: str, rgb: np.ndarray) -> None:
        with self._lock:
            self.frames[name] = rgb
            self.updated[name] = time.monotonic()
            self.status[name] = ""
            root = self._root
            if root is None:
                return
            slot = self._writers.get(name)
            if slot is None:
                folder = root / name
                folder.mkdir(parents=True, exist_ok=True)
                times = (folder / "times.csv").open("w", encoding="utf-8")
                times.write("index,unix\n")
                slot = [times, 0, folder]
                self._writers[name] = slot
            index = slot[1]
            try:
                save_jpeg(slot[2] / f"{index:06d}.jpg", rgb)
            except Exception as exc:
                print(f"[lerobot-cam] jpeg {name}: {exc}", file=sys.stderr, flush=True)
                return
            slot[0].write(f"{index},{time.time():.6f}\n")
            slot[0].flush()
            slot[1] += 1

    def _close_writers(self) -> None:
        for slot in self._writers.values():
            slot[0].close()
        self._writers.clear()


def _ppm(rgb: np.ndarray) -> bytes:
    height, width = rgb.shape[:2]
    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    return header + np.ascontiguousarray(rgb).tobytes()


def stream_v4l2(spec: CameraSpec, device: str, hub: ViewHub, stop: threading.Event) -> None:
    ffmpeg = _ffmpeg_bin()
    if ffmpeg is None:
        hub.set_status(spec.name, "找不到 ffmpeg，打不开彩色画面。")
        return
    command = [
        ffmpeg,
        "-loglevel",
        "error",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-f",
        "v4l2",
        "-input_format",
        "mjpeg",
        "-video_size",
        f"{WIDTH}x{HEIGHT}",
        "-framerate",
        str(FPS),
        "-i",
        device,
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "pipe:1",
    ]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    frame_bytes = WIDTH * HEIGHT * 3
    print(f"[lerobot-cam] view {spec.name} {device}", flush=True)
    try:
        while not stop.is_set():
            if proc.stdout is None:
                break
            buf = bytearray()
            while len(buf) < frame_bytes and not stop.is_set():
                chunk = proc.stdout.read(frame_bytes - len(buf))
                if not chunk:
                    hub.set_status(spec.name, f"彩色画面中断（{device}）")
                    return
                buf.extend(chunk)
            rgb = np.frombuffer(bytes(buf), dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
            hub.push(spec.name, rgb.copy())
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            proc.kill()


def stream_one(spec: CameraSpec, hub: ViewHub, stop: threading.Event, sdk) -> None:
    pipeline = None
    try:
        pipeline = open_color_pipeline(sdk, spec.serial)
        print(f"[lerobot-cam] view {spec.name} serial={spec.serial}", flush=True)
        period = 1.0 / FPS
        next_t = time.monotonic()
        while not stop.is_set():
            frames = pipeline.wait_for_frames(200)
            if frames is None:
                continue
            color = frames.get_color_frame()
            if not color:
                continue
            now = time.monotonic()
            if now < next_t:
                continue
            next_t = now + period
            hub.push(spec.name, resize_rgb(decode_color(color), WIDTH, HEIGHT))
    except Exception as exc:
        hub.set_status(spec.name, str(exc))
        print(f"[lerobot-cam] view {spec.name}: {exc}", file=sys.stderr, flush=True)
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass


def _serve_view(server: socket.socket, hub: ViewHub, stop: threading.Event) -> None:
    server.settimeout(0.3)
    while not stop.is_set():
        try:
            conn, _addr = server.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            conn.settimeout(2.0)
            data = b""
            while b"\n" not in data and len(data) < 1024:
                chunk = conn.recv(256)
                if not chunk:
                    break
                data += chunk
            line = data.decode("utf-8", errors="replace").strip()
            if line == "ping":
                conn.sendall(b"ok\n")
            elif line == "snapshot head":
                frame, status = hub.latest("head")
                if frame is None:
                    conn.sendall(b"err " + (status or "等待头部相机").encode("utf-8") + b"\n")
                else:
                    from PIL import Image

                    output = io.BytesIO()
                    Image.fromarray(frame).save(output, format="JPEG", quality=70)
                    jpeg = output.getvalue()
                    conn.sendall(f"jpeg {len(jpeg)}\n".encode("ascii") + jpeg)
            elif line.startswith("record "):
                hub.begin(Path(line.split(" ", 1)[1].strip()))
                conn.sendall(b"ok\n")
            elif line == "release":
                hub.end()
                conn.sendall(b"ok\n")
            else:
                conn.sendall(b"err\n")
        except Exception as exc:
            print(f"[lerobot-cam] view command failed: {exc}", file=sys.stderr, flush=True)
        finally:
            conn.close()
    server.close()
    hub.end()


def view(headless: bool = False) -> int:
    stop = threading.Event()
    hub = ViewHub()

    def _stop(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        import pyorbbecsdk as sdk
    except ImportError:
        sdk = None
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(("127.0.0.1", VIEW_PORT))
        server.listen(2)
    except OSError as exc:
        print(f"[lerobot-cam] view port {VIEW_PORT} is already in use: {exc}", file=sys.stderr)
        server.close()
        return 1
    threading.Thread(target=_serve_view, args=(server, hub, stop), daemon=True).start()
    specs = apply_usb_ports(load_specs(default_config()))
    if sdk is not None:
        specs = bind_serials(specs, list_serials(sdk))
    for spec in specs:
        if not spec.usb and not spec.serial and not (spec.name == "head" and os.environ.get("OPENARM_CAMERA_HEAD_DEVICE")):
            hub.set_status(spec.name, "这个位置没有选 USB 口。")
            continue
        port = spec.usb or port_for_serial(spec.serial)
        device = os.environ.get("OPENARM_CAMERA_HEAD_DEVICE", "") if spec.name == "head" else ""
        device = device or color_device_for_port(port)
        if device:
            threading.Thread(target=stream_v4l2, args=(spec, device, hub, stop), daemon=True).start()
            continue
        if sdk is not None and spec.serial:
            threading.Thread(target=stream_one, args=(spec, hub, stop, sdk), daemon=True).start()
            continue
        if sdk is None:
            hub.set_status(spec.name, "这个 USB 口没有彩色画面，而且没有 pyorbbecsdk。")
        else:
            hub.set_status(spec.name, "没有打开这台相机。")

    if headless:
        print("[lerobot-cam] shared camera capture ready (headless)", flush=True)
        while not stop.wait(0.2):
            pass
        server.close()
        hub.end()
        return 0

    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("相机")
    root.geometry("320x90")
    ttk.Label(root, text="头部、左腕、右腕各一个窗口，可以拖开。").pack(padx=12, pady=16)
    windows = {}
    for index, name in enumerate(NAMES):
        win = tk.Toplevel(root)
        win.title(VIEW_TITLES.get(name, name))
        win.geometry(f"660x520+{60 + index * 48}+{80 + index * 48}")
        label = ttk.Label(win, text="正在打开…", anchor="center")
        label.pack(fill="both", expand=True)
        windows[name] = {"label": label, "photo": None}

        def _close(event_name=name, event_win=win):
            event_win.destroy()

        win.protocol("WM_DELETE_WINDOW", _close)

    def refresh() -> None:
        if stop.is_set() or not root.winfo_exists():
            root.destroy()
            return
        for name, ui in windows.items():
            if not ui["label"].winfo_exists():
                continue
            frame, text = hub.latest(name)
            if frame is None:
                ui["label"].configure(image="", text=text or "等待画面")
                continue
            try:
                preview = Path(f"/tmp/openarm_preview_{name}.ppm")
                preview.write_bytes(_ppm(frame))
                photo = tk.PhotoImage(file=str(preview))
            except tk.TclError as exc:
                ui["label"].configure(image="", text=str(exc))
                continue
            ui["photo"] = photo
            ui["label"].configure(image=photo, text="")
        root.after(33, refresh)

    root.protocol("WM_DELETE_WINDOW", lambda: stop.set())
    root.after(33, refresh)
    root.mainloop()
    stop.set()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Record or preview Gemini 335 color")
    parser.add_argument("command", choices=("record", "view"))
    parser.add_argument("--out", type=Path)
    parser.add_argument("--headless", action="store_true", help="share camera frames without desktop windows")
    args = parser.parse_args()
    if args.command == "view":
        return view(headless=args.headless)
    if args.out is None:
        print("[lerobot-cam] record needs --out", file=sys.stderr)
        return 2
    return record(args.out)


if __name__ == "__main__":
    raise SystemExit(main())

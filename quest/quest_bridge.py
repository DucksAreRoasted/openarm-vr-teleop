#!/usr/bin/env python3
"""Quest WebXR bridge: forwards controller poses to OpenArm Cartesian control via UDP.

Start this on the control PC, then run:
  ./build/quest_cartesian_control --sim --bimanual

UDP payload format (CSV):
  timestamp_ms,grip,trigger,px,py,pz,qx,qy,qz,qw,btn_close,btn_open
  btn_close = A (right) / X (left); btn_open = B (right) / Y (left)

Right controller -> --udp-right-port (default 47911)
Left controller  -> --udp-left-port  (default 47912)
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import socket
import ssl
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Optional
from urllib.parse import urlparse

import websockets
from body_tracking import skeleton_packet, skeleton_diagnostics
from capture_shortcuts import CaptureShortcuts


class ThreadingHTTPSServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class HTTPRequestHandler(BaseHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass

    def do_OPTIONS(self) -> None:
        self.send_response(200)
        self.end_headers()

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            path = "/client.html"
        if path == "/tracking_config":
            body = json.dumps({"mode": os.environ.get("VR_TELEOP_MODE", "human_mimic")}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Content-length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        file_path = os.path.join(os.path.dirname(__file__), path.lstrip("/"))
        if os.path.exists(file_path) and os.path.isfile(file_path):
            import mimetypes

            content_type, _ = mimetypes.guess_type(file_path)
            if content_type is None:
                content_type = "application/octet-stream"
            with open(file_path, "rb") as handle:
                content = handle.read()
            self.send_response(200)
            self.send_header("Content-type", content_type)
            self.send_header("Content-length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)
        else:
            self.send_error(404, f"File not found: {path}")

    def log_message(self, format: str, *args) -> None:
        pass


class QuestUdpForwarder:
    def __init__(self, host: str, port: int, stats: "HandBridgeStats") -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (host, port)
        self.stats = stats

    def send_controller(self, controller: Optional[dict]) -> None:
        if not controller:
            return
        position = controller.get("position", {})
        quaternion = controller.get("quaternion")
        if quaternion is None:
            self.stats.skipped_no_quaternion += 1
            return
        payload = ",".join(
            [
                str(controller.get("timestamp", 0)),
                "1" if controller.get("gripActive", False) else "0",
                f"{float(controller.get('trigger', 0.0)):.6f}",
                f"{float(position.get('x', 0.0)):.6f}",
                f"{float(position.get('y', 0.0)):.6f}",
                f"{float(position.get('z', 0.0)):.6f}",
                f"{float(quaternion.get('x', 0.0)):.6f}",
                f"{float(quaternion.get('y', 0.0)):.6f}",
                f"{float(quaternion.get('z', 0.0)):.6f}",
                f"{float(quaternion.get('w', 1.0)):.6f}",
                "1" if controller.get("buttonClose", False) else "0",
                "1" if controller.get("buttonOpen", False) else "0",
            ]
        )
        self.sock.sendto(payload.encode("ascii"), self.target)
        self.stats.udp_sent += 1
        self.stats.on_udp_sent()
        self.stats.last_grip = bool(controller.get("gripActive", False))
        self.stats.last_trigger = float(controller.get("trigger", 0.0))
        self.stats.last_btn_close = bool(controller.get("buttonClose", False))
        self.stats.last_btn_open = bool(controller.get("buttonOpen", False))
        self.stats.last_pos = (
            float(position.get("x", 0.0)),
            float(position.get("y", 0.0)),
            float(position.get("z", 0.0)),
        )


class HandBridgeStats:
    def __init__(self, hand: str) -> None:
        self.hand = hand
        self.ws_messages = 0
        self.udp_sent = 0
        self.skipped_no_quaternion = 0
        self.last_grip = False
        self.last_trigger = 0.0
        self.last_btn_close = False
        self.last_btn_open = False
        self.last_pos = (0.0, 0.0, 0.0)
        self._window_udp = 0
        self._last_report = time.monotonic()

    def on_udp_sent(self) -> None:
        self._window_udp += 1

    def maybe_report(self, client_connected: bool) -> None:
        now = time.monotonic()
        if now - self._last_report < 1.0:
            return
        elapsed = now - self._last_report
        udp_hz = self._window_udp / elapsed
        status = "connected" if client_connected else "waiting"
        pos = self.last_pos
        print(
            f"[quest-bridge] hand={self.hand} client={status} udp={udp_hz:.1f}Hz "
            f"grip={int(self.last_grip)} trigger={self.last_trigger:.2f} "
            f"btn_close={int(self.last_btn_close)} btn_open={int(self.last_btn_open)} "
            f"pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] "
            f"skip_no_quat={self.skipped_no_quaternion}",
            flush=True,
        )
        self._window_udp = 0
        self._last_report = now


class QuestBridge:
    def __init__(
        self,
        host: str = "0.0.0.0",
        http_port: int = 8443,
        ws_port: int = 8442,
        udp_host: str = "127.0.0.1",
        udp_right_port: int = 47911,
        udp_left_port: int = 47912,
        bimanual: bool = True,
        log_interval: float = 1.0,
    ) -> None:
        self.host = host
        self.http_port = http_port
        self.ws_port = ws_port
        self.log_interval = log_interval
        self.bimanual = bimanual
        self.client_connected = False
        self.right_stats = HandBridgeStats("right")
        self.left_stats = HandBridgeStats("left")
        self.right_forwarder = QuestUdpForwarder(udp_host, udp_right_port, self.right_stats)
        self.left_forwarder = QuestUdpForwarder(udp_host, udp_left_port, self.left_stats)
        self.skeleton_sock = None
        self.skeleton_target = (udp_host, int(os.environ.get("OPENARM_SKELETON_POSE_PORT", "47951")))
        if os.environ.get("VR_TELEOP_MODE") == "skeleton_proximal":
            self.skeleton_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ssl_context = create_self_signed_cert()
        self.http_server: Optional[HTTPServer] = None
        self.ws_clients: set = set()
        self.grip_telem_port = int(os.environ.get("OPENARM_QUEST_GRIP_TELEM_PORT", "47945"))
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def start_http_server(self) -> bool:
        if self.ssl_context is None:
            return False
        self.http_server = ThreadingHTTPSServer((self.host, self.http_port), HTTPRequestHandler)
        self.http_server.socket = self.ssl_context.wrap_socket(
            self.http_server.socket, server_side=True
        )
        thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        thread.start()
        print(f"HTTPS server: https://{self.host}:{self.http_port}/client.html")
        return True

    async def handle_client(self, websocket) -> None:
        self.client_connected = True
        self.ws_clients.add(websocket)
        print(f"Quest client connected: {websocket.remote_address}", flush=True)
        last_timestamp = -1
        last_skeleton_status = None
        last_skeleton_log = 0.0
        shortcuts = CaptureShortcuts()
        capture_lock = asyncio.Lock()
        capture_tasks = set()
        try:
            async for message in websocket:
                data = json.loads(message)
                timestamp = data.get("timestamp", 0)
                if type(timestamp) not in (int, float) or not math.isfinite(timestamp) or timestamp <= last_timestamp:
                    continue
                last_timestamp = timestamp
                if self.skeleton_sock is not None:
                    body = data.get("bodyTracking")
                    self.skeleton_sock.sendto(skeleton_packet(body, timestamp), self.skeleton_target)
                    status = skeleton_diagnostics(body, timestamp)
                    now = time.monotonic()
                    if status != last_skeleton_status or now-last_skeleton_log >= 2.0:
                        print(f"[quest-bridge][skeleton] left={status[0]} right={status[1]} "
                              f"target={self.skeleton_target[0]}:{self.skeleton_target[1]}", flush=True)
                        last_skeleton_status, last_skeleton_log = status, now
                right = data.get("rightController")
                left = data.get("leftController")
                for hand, controller in (("left", left), ("right", right)):
                    action = shortcuts.update(hand, controller, time.monotonic())
                    if action:
                        if capture_tasks:
                            await websocket.send(json.dumps({"type": "capture_feedback", "action": action,
                                "ok": False, "message": "正在处理上一次采集操作，请稍后再按。"}, ensure_ascii=False))
                        else:
                            task = asyncio.create_task(self.capture_shortcut(websocket, action, capture_lock))
                            capture_tasks.add(task)
                            task.add_done_callback(capture_tasks.discard)
                if right:
                    right["timestamp"] = timestamp
                    self.right_forwarder.send_controller(right)
                if self.bimanual and left:
                    left["timestamp"] = timestamp
                    self.left_forwarder.send_controller(left)
                self.right_stats.maybe_report(self.client_connected)
                if self.bimanual:
                    self.left_stats.maybe_report(self.client_connected)
        except websockets.exceptions.ConnectionClosed:
            print("Quest client disconnected", flush=True)
        finally:
            for task in capture_tasks:
                task.cancel()
            self.ws_clients.discard(websocket)
            self.client_connected = bool(self.ws_clients)

    async def capture_shortcut(self, websocket, action: str, lock: asyncio.Lock) -> None:
        async with lock:
            # A separate socket per request prevents late replies acknowledging another click.
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            loop = asyncio.get_running_loop()
            payload = {"type": "capture_feedback", "action": action, "ok": False,
                       "message": "未收到采集反馈，请在 ./app.sh 控制台核对状态。"}
            try:
                target = ("127.0.0.1", int(os.environ.get("OPENARM_QUEST_CAPTURE_PORT", "47934")))
                sock.connect(target)
                await loop.sock_sendall(sock, json.dumps({"action": action}).encode())
                response = await asyncio.wait_for(loop.sock_recv(sock, 4096), timeout=.8)
                reply = json.loads(response)
                payload.update(ok=reply.get("ok") is True, message=str(reply.get("message", "")))
            except (OSError, ValueError, asyncio.TimeoutError):
                pass
            finally:
                sock.close()
            print(f"[quest-capture] {action}: {payload['message']}", flush=True)
            try:
                await websocket.send(json.dumps(payload, ensure_ascii=False))
            except websockets.exceptions.ConnectionClosed:
                pass

    async def broadcast_json(self, payload: dict) -> None:
        if not self.ws_clients:
            return
        text = json.dumps(payload, separators=(",", ":"))
        dead = []
        for ws in list(self.ws_clients):
            try:
                await ws.send(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.ws_clients.discard(ws)

    def _on_grip_telem_datagram(self, data: bytes, _addr) -> None:
        # CSV: t,L_tau,L_mos,L_rot,L_q,R_tau,R_mos,R_rot,R_q
        try:
            parts = data.decode("utf-8", errors="replace").strip().split(",")
            if len(parts) < 9:
                return
            vals = [float(x) for x in parts[:9]]
            payload = {
                "type": "gripper_telem",
                "t": vals[0],
                "L": {"tau": vals[1], "tmos": int(vals[2]), "trotor": int(vals[3]), "q": vals[4]},
                "R": {"tau": vals[5], "tmos": int(vals[6]), "trotor": int(vals[7]), "q": vals[8]},
            }
        except (ValueError, UnicodeError):
            return
        if self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(self.broadcast_json(payload), self._loop)

    def start_grip_telem_listener(self) -> None:
        if self.grip_telem_port <= 0:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", self.grip_telem_port))
        except OSError as exc:
            print(f"[quest-bridge] grip telem bind :{self.grip_telem_port} failed: {exc}", flush=True)
            sock.close()
            return
        sock.settimeout(0.5)
        print(f"[quest-bridge] grip telem UDP listen 127.0.0.1:{self.grip_telem_port}", flush=True)

        def _loop() -> None:
            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    self._on_grip_telem_datagram(data, addr)
                except socket.timeout:
                    continue
                except OSError:
                    break

        threading.Thread(target=_loop, name="grip-telem-udp", daemon=True).start()

    async def stats_reporter(self) -> None:
        while True:
            await asyncio.sleep(self.log_interval)
            self.right_stats.maybe_report(self.client_connected)
            if self.bimanual:
                self.left_stats.maybe_report(self.client_connected)

    async def run(self) -> None:
        self._loop = asyncio.get_running_loop()
        try:
            await self._run_servers()
        finally:
            if self.skeleton_sock is not None:
                self.skeleton_sock.close()
            if self.http_server is not None:
                self.http_server.shutdown()
                self.http_server.server_close()

    async def _run_servers(self) -> None:
        self.start_grip_telem_listener()
        if not self.start_http_server():
            raise RuntimeError("Failed to start HTTPS server")
        asyncio.create_task(self.stats_reporter())
        async with websockets.serve(
            self.handle_client, self.host, self.ws_port, ssl=self.ssl_context
        ):
            print(f"WebSocket server: wss://{self.host}:{self.ws_port}", flush=True)
            print(
                f"Forwarding right-controller -> UDP "
                f"{self.right_forwarder.target[0]}:{self.right_forwarder.target[1]}",
                flush=True,
            )
            if self.bimanual:
                print(
                    f"Forwarding left-controller  -> UDP "
                    f"{self.left_forwarder.target[0]}:{self.left_forwarder.target[1]}",
                    flush=True,
                )
            print("[quest-bridge] waiting for Quest data... (stats every 1s)", flush=True)
            await asyncio.Future()


def create_self_signed_cert() -> Optional[ssl.SSLContext]:
    cert_path = os.path.join(os.path.dirname(__file__), "quest_cert.pem")
    key_path = os.path.join(os.path.dirname(__file__), "quest_key.pem")
    if not os.path.exists(cert_path):
        print("Generating SSL certificate...")
        import socket as pysocket

        hostname = pysocket.gethostname()
        ips = ["127.0.0.1", "0.0.0.0"]
        try:
            probe = pysocket.socket(pysocket.AF_INET, pysocket.SOCK_DGRAM)
            probe.connect(("8.8.8.8", 80))
            ips.append(probe.getsockname()[0])
            probe.close()
        except OSError:
            pass
        alt_names = "\n".join(f"IP.{index + 1} = {ip}" for index, ip in enumerate(ips))
        config = f"""
[req]
distinguished_name = req_distinguished_name
x509_extensions = v3_req
prompt = no
[req_distinguished_name]
CN = {hostname}
[v3_req]
basicConstraints = CA:FALSE
subjectAltName = @alt_names
[alt_names]
{alt_names}
"""
        try:
            subprocess.run(
                ["openssl", "genrsa", "-out", key_path, "2048"],
                check=True,
                stderr=subprocess.DEVNULL,
            )
            with tempfile.NamedTemporaryFile("w", delete=False) as handle:
                handle.write(config)
                config_path = handle.name
            subprocess.run(
                [
                    "openssl",
                    "req",
                    "-new",
                    "-x509",
                    "-key",
                    key_path,
                    "-out",
                    cert_path,
                    "-days",
                    "365",
                    "-config",
                    config_path,
                ],
                check=True,
                stderr=subprocess.DEVNULL,
            )
            os.unlink(config_path)
        except Exception as exc:
            print(f"Cert generation failed: {exc}")
            return None

    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    try:
        context.load_cert_chain(cert_path, key_path)
        return context
    except Exception as exc:
        print(f"Failed to load cert: {exc}")
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Quest to OpenArm UDP bridge")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8443)
    parser.add_argument("--ws-port", type=int, default=8442)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=47911,
                        help="right-hand UDP port (single-arm legacy)")
    parser.add_argument("--udp-right-port", type=int, default=47911)
    parser.add_argument("--udp-left-port", type=int, default=47912)
    parser.add_argument("--single-arm", action="store_true",
                        help="only forward right controller")
    parser.add_argument("--log-interval", type=float, default=1.0,
                        help="stats print interval in seconds")
    args = parser.parse_args()

    bimanual = not args.single_arm
    bridge = QuestBridge(
        host=args.host,
        http_port=args.http_port,
        ws_port=args.ws_port,
        udp_host=args.udp_host,
        udp_right_port=args.udp_right_port if bimanual else args.udp_port,
        udp_left_port=args.udp_left_port,
        bimanual=bimanual,
        log_interval=args.log_interval,
    )
    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        print("Stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

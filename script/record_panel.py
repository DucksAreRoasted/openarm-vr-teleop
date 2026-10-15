#!/usr/bin/env python3
"""Local page for labeling a teleop take.

The controller listens on UDP. This page only forwards button presses:

    start,<name>   begin a take
    stop           end it and wait for a label
    success        keep the take as a successful episode
    fail           keep the take as a failed episode
    redo           move the take to trash so the next start reuses its number
    undo           restore that take within 10 seconds

Bind is localhost. Open http://127.0.0.1:8765 while teleop is running.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socket
from urllib.parse import parse_qs

PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>录制</title>
<style>
  body { font-family: sans-serif; margin: 2rem auto; max-width: 36rem; color: #1c1c1c; }
  h1 { font-size: 1.25rem; }
  label { display: block; margin: 1rem 0 0.35rem; }
  input { width: 100%; box-sizing: border-box; font: inherit; padding: 0.45rem 0.6rem; }
  .row { display: flex; gap: 0.6rem; margin-top: 0.8rem; }
  button { flex: 1; font: inherit; padding: 0.7rem 0.4rem; cursor: pointer; }
  button:disabled { cursor: default; opacity: 0.45; }
  #ok { background: #e7f6e7; }
  #bad { background: #fde8e8; }
  #status { margin-top: 1rem; }
</style>
</head>
<body>
  <h1>数据采集</h1>
  <p>停下来之后，给这一条打成功或失败。不满意就丢弃；10 秒内可以撤销，超时后留在回收站。</p>
  <label for="name">批次名称</label>
  <input id="name" value="">
  <div class="row">
    <button id="start">开始录制</button>
    <button id="stop" disabled>停止</button>
  </div>
  <div class="row">
    <button id="ok" disabled>成功</button>
    <button id="bad" disabled>失败</button>
    <button id="redo" disabled>重新录制</button>
  </div>
  <p id="status">还没开始。</p>
<script>
const name = document.getElementById("name");
const start = document.getElementById("start");
const stop = document.getElementById("stop");
const ok = document.getElementById("ok");
const bad = document.getElementById("bad");
const redo = document.getElementById("redo");
const status = document.getElementById("status");
let undoTimer = 0;
const stamp = new Date();
const pad = (n) => String(n).padStart(2, "0");
name.value = "session_" + stamp.getFullYear() + pad(stamp.getMonth()+1) + pad(stamp.getDate())
  + "_" + pad(stamp.getHours()) + pad(stamp.getMinutes()) + pad(stamp.getSeconds());

function show(text) { status.textContent = text; }
function buttons({ recording, waiting, canRedo }) {
  start.disabled = recording || waiting;
  stop.disabled = !recording;
  ok.disabled = !waiting;
  bad.disabled = !waiting;
  redo.disabled = !canRedo;
  name.disabled = recording || waiting;
}
async function send(command) {
  const response = await fetch("/command", {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body: "command=" + encodeURIComponent(command),
  });
  if (!response.ok) throw new Error(await response.text());
}
function clearUndo() {
  clearTimeout(undoTimer);
  redo.dataset.undo = "";
  redo.textContent = "重新录制";
}
start.onclick = async () => {
  const batch = name.value.trim();
  if (!batch) { show("先填批次名称。"); return; }
  try {
    await send("start," + batch);
    clearUndo();
    buttons({ recording: true, waiting: false, canRedo: false });
    show("正在录制 " + batch + "。做完点停止。");
  } catch (error) { show(String(error)); }
};
stop.onclick = async () => {
  try {
    await send("stop");
    buttons({ recording: false, waiting: true, canRedo: true });
    show("已停止。这条算成功、失败，还是重录？");
  } catch (error) { show(String(error)); }
};
ok.onclick = async () => {
  try {
    await send("success");
    buttons({ recording: false, waiting: false, canRedo: true });
    show("已标为成功，写入数据集。下一条可以开始，或重录刚刚这一条。");
  } catch (error) { show(String(error)); }
};
bad.onclick = async () => {
  try {
    await send("fail");
    buttons({ recording: false, waiting: false, canRedo: true });
    show("已标为失败，写入数据集。下一条可以开始，或重录刚刚这一条。");
  } catch (error) { show(String(error)); }
};
redo.onclick = async () => {
  try {
    if (redo.dataset.undo === "1") {
      await send("undo");
      const waiting = redo.dataset.waiting === "1";
      clearUndo();
      buttons({ recording: false, waiting: waiting, canRedo: true });
      show(waiting ? "已恢复这一条。请标成功、失败，或再次丢弃。"
                   : "已恢复这一条。可以开始下一条，或再次丢弃。");
      return;
    }
    const waiting = !ok.disabled;
    await send("redo");
    buttons({ recording: false, waiting: false, canRedo: false });
    redo.dataset.undo = "1";
    redo.dataset.waiting = waiting ? "1" : "0";
    redo.textContent = "撤销丢弃";
    redo.disabled = false;
    clearTimeout(undoTimer);
    undoTimer = setTimeout(() => {
      clearUndo();
      redo.disabled = true;
      show("撤销时间已过，这一条留在回收站。点开始会用同一个序号。");
    }, 10000);
    show("这一条已移到回收站。10 秒内可点「撤销丢弃」恢复。");
  } catch (error) { show(String(error)); }
};
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    udp_port = 47930

    def do_GET(self) -> None:  # noqa: N802
        if self.path.split("?", 1)[0] != "/":
            self.send_error(404)
            return
        body = PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:  # noqa: N802
        if self.path.split("?", 1)[0] != "/command":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8", errors="replace")
        command = parse_qs(raw).get("command", [""])[0].strip()
        if not command or len(command) > 200:
            self.send_error(400, "bad command")
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(command.encode("utf-8"), ("127.0.0.1", self.udp_port))
        finally:
            sock.close()
        body = b"ok"
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser(description="Local record/label page")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--udp-port", type=int, default=47930)
    args = parser.parse_args()
    Handler.udp_port = args.udp_port
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[record-panel] http://{args.host}:{args.port}  -> UDP {args.udp_port}", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

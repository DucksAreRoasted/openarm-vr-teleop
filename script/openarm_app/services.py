"""Shared helpers for CAN, recording, and process control."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import socket
import subprocess

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "run.sh"
CAN_HELPER = ROOT / "scripts" / "configure_can.sh"
RECORD_PORT = int(os.environ.get("OPENARM_QUEST_UI_RECORD_PORT", "47930"))
HTTP_PORT = int(os.environ.get("OPENARM_QUEST_HTTP_PORT", "8443"))
GRAVITY_YAML = ROOT / "config" / "gravity_comp.yaml"


def quest_udp_host() -> str:
    """IPv4 address shared with the Quest bridge. Default is loopback."""
    host = os.environ.get("OPENARM_QUEST_UDP_HOST", "").strip()
    return host or "127.0.0.1"


def sudo_is_cached() -> bool:
    result = subprocess.run(
        ["sudo", "-n", "true"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def configure_can(
    left: str, right: str, bitrate: str, dbitrate: str, password: str | None
) -> None:
    if not CAN_HELPER.is_file():
        raise FileNotFoundError(f"CAN 配置脚本不存在：{CAN_HELPER}")
    cached = sudo_is_cached()
    command = [
        "sudo",
        "-n" if cached else "-S",
        "-p",
        "",
        "bash",
        str(CAN_HELPER),
        left,
        right,
        bitrate,
        dbitrate,
    ]
    result = subprocess.run(
        command,
        input=None if cached else (password or "") + "\n",
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip().splitlines()
        reason = detail[-1] if detail else f"sudo 返回 {result.returncode}"
        raise RuntimeError(f"CAN 配置失败：{reason}")


def local_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def camera_view_open() -> bool:
    try:
        sock = socket.create_connection(("127.0.0.1", 47932), timeout=0.3)
    except OSError:
        return False
    try:
        sock.sendall(b"ping\n")
        return sock.recv(16).startswith(b"ok")
    except OSError:
        return False
    finally:
        sock.close()


def send_record(command: str) -> None:
    payload = command.strip().encode("utf-8")
    if not payload:
        raise ValueError("空指令")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(payload, ("127.0.0.1", RECORD_PORT))
    finally:
        sock.close()


def sighup_gravity_processes() -> tuple[bool, int]:
    pids: set[int] = set()
    for args in (
        ["pgrep", "-x", "openarm_zero_g"],
        ["pgrep", "-f", "/openarm_zero_g"],
        ["pgrep", "-f", "/quest_cartesian_control .*--mirror-teach"],
        ["pgrep", "-f", "/quest_cartesian_control .*--ker( |$)"],
        ["pgrep", "-f", "openarm_moveit_config/real_arm"],
    ):
        result = subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
        for token in (result.stdout or "").split():
            if token.isdigit():
                pids.add(int(token))
    pids.discard(os.getpid())
    sent = 0
    for pid in pids:
        try:
            os.kill(pid, signal.SIGHUP)
            sent += 1
        except OSError:
            pass
    return sent > 0, sent


def resolve_gravity_yaml() -> Path:
    env_yaml = os.environ.get("OPENARM_GRAVITY_YAML", "").strip()
    if env_yaml:
        return Path(env_yaml).expanduser().resolve()
    env_root = os.environ.get("OPENARM_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve() / "config" / "gravity_comp.yaml"
    return GRAVITY_YAML

#!/usr/bin/env python3
"""Read OpenArm KER encoders and send calibrated V1 joint targets over loopback UDP."""
from __future__ import annotations

import argparse
import errno
import json
import math
from pathlib import Path
import signal
import socket
import statistics
import sys
import tempfile
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def ker_python() -> str:
    import os
    override = os.environ.get("OPENARM_KER_PYTHON")
    if override:
        return override
    dedicated = ROOT / ".venv-ker/bin/python"
    return str(dedicated) if dedicated.is_file() else sys.executable


def load_mapping(path: Path, *, real: bool = False) -> dict:
    with path.open(encoding="utf-8") as handle:
        mapping = yaml.safe_load(handle)
    if not isinstance(mapping, dict):
        raise ValueError("KER 映射配置必须是字典")
    if real and mapping.get("calibrated") is not True:
        raise ValueError("请先在仿真核对 KER→V1 的方向、零点及夹爪行程，再将配置 calibrated 设为 true")
    used = []
    for side in ("left", "right"):
        arm = mapping.get(side)
        if not isinstance(arm, dict):
            raise ValueError(f"KER 配置缺少 {side}")
        for name in ("channels", "signs", "zero_degrees", "offset_rad"):
            values = arm.get(name)
            if not isinstance(values, list) or len(values) != 7:
                raise ValueError(f"{side}.{name} 必须包含七项")
        channels = [*arm["channels"], arm.get("gripper_channel")]
        if any(type(channel) is not int or not 0 <= channel < 16 for channel in channels):
            raise ValueError(f"{side}: 通道必须在 0～15 之间")
        used.extend(channels)
        if any(type(sign) not in (int, float) or sign not in (-1, 1) for sign in arm["signs"]):
            raise ValueError(f"{side}.signs 只能是 -1 或 1")
        values = [*arm["zero_degrees"], *arm["offset_rad"],
                  arm.get("gripper_open_degrees"), arm.get("gripper_closed_degrees")]
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise ValueError(f"{side}: 零点、偏置和夹爪端点必须是有限数值")
        if arm["gripper_open_degrees"] == arm["gripper_closed_degrees"]:
            raise ValueError(f"{side}: 夹爪两端不能相同")
    if len(set(used)) != 16:
        raise ValueError("KER 通道不得重复")
    return mapping


def map_angles(raw, mapping: dict) -> list[float]:
    if len(raw) != 16 or any(not math.isfinite(float(value)) for value in raw):
        raise ValueError("KER 必须提供十六个有限编码器角度")
    result = []
    for side in ("left", "right"):
        arm = mapping[side]
        for channel, sign, zero, offset in zip(arm["channels"], arm["signs"],
                                               arm["zero_degrees"], arm["offset_rad"]):
            result.append(sign * math.radians(float(raw[channel])-zero) + offset)
        opening, closing = arm["gripper_open_degrees"], arm["gripper_closed_degrees"]
        ratio = (float(raw[arm["gripper_channel"]])-opening)/(closing-opening)
        # Saturate the calibrated gripper gesture only; arm limits are checked in C++.
        result.append(-.98 + 1.6*max(0., min(1., ratio)))
    return result


class KerDeviceInput:
    """Validate encoder health and device freshness before creating a UDP heartbeat."""
    def __init__(self):
        self.sequence = None
        self.timestamp = None
        self.duplicate_frames = 0

    def accept(self, frame: dict):
        mask = frame.get("error_mask")
        if mask is not None:
            if type(mask) is not int or not 0 <= mask <= 65535:
                raise ValueError("KER error_mask 格式无效")
            faults = [i for i in range(16) if mask & (1 << i)]
        else:
            errors = frame.get("errors")
            if errors is not None and (not isinstance(errors, list) or len(errors) != 16
                    or any(type(v) not in (bool, int) or v not in (0, 1) for v in errors)):
                raise ValueError("KER errors 格式无效")
            faults = [i for i, value in enumerate(errors or []) if value]
        if faults:
            raise RuntimeError(f"KER 编码器故障通道（从 0 编号）: {faults}；"
                               "设备可能保持旧角度，请检查 KER 编码器连接及供电后重启；未发送控制命令")
        seq = frame.get("seq")
        if seq is not None:
            if type(seq) is not int or not 0 <= seq <= 0xffffffff:
                raise ValueError("KER 设备序号无效")
            if self.sequence is not None:
                delta = (seq-self.sequence) & 0xffffffff
                if delta == 0:
                    self.duplicate_frames += 1
                    return None
                if delta >= 0x80000000:
                    raise RuntimeError("KER 设备序号倒退或设备重启，请重新启动遥操")
            self.sequence = seq
        # FW 2.0.0 uses timestamp rather than seq. Identical positions are valid
        # while held still, but repeated device timestamps are not new samples.
        if seq is None and "timestamp" in frame:
            stamp = frame["timestamp"]
            if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
                raise ValueError("KER 设备时间戳无效")
            if stamp == self.timestamp:
                self.duplicate_frames += 1
                return None
            # Device timestamp units are firmware-specific; use only equality.
            # PC watchdog time remains monotonic and independent of device units.
            self.timestamp = stamp
        angles = frame.get("angles")
        if angles is None:
            compact = frame.get("angles_cd")
            if not isinstance(compact, list) or len(compact) != 16:
                raise ValueError("KER 缺少十六通道角度数据")
            angles = [v/100. for v in compact]
        return {**frame, "angles":angles}


def trigger_pressed(raw, mapping: dict, side: str) -> bool:
    arm = mapping[side]
    opening, closing = arm["gripper_open_degrees"], arm["gripper_closed_degrees"]
    ratio = (float(raw[arm["gripper_channel"]])-opening)/(closing-opening)
    return math.isfinite(ratio) and ratio >= .9


def trigger_endpoints(mapping: dict, released: list, pressed: list) -> dict:
    """Measure both endpoints explicitly; never infer them from a startup pose."""
    import copy
    result = copy.deepcopy(mapping)
    for side in ("left", "right"):
        channel = mapping[side]["gripper_channel"]
        endpoints = []
        for samples in (released, pressed):
            if len(samples) < 5:
                raise ValueError("每个端点至少需要五个新鲜样本")
            values = [float(raw[channel]) for raw in samples]
            if any(not math.isfinite(v) for v in values) or max(values)-min(values) > 2:
                raise ValueError(f"{side} 扳机读数不稳定，请保持不动后重新采集")
            endpoints.append(statistics.median(values))
        if abs(endpoints[1]-endpoints[0]) < 5:
            raise ValueError(f"{side} 扳机行程小于 5°；请确认按到底且该通道读数随扳机变化")
        result[side]["gripper_open_degrees"], result[side]["gripper_closed_degrees"] = endpoints
    # Arm direction/zero calibration is independent of trigger calibration.
    return result


def inspect_stream(stream, running, *, output=None) -> None:
    """Read-only calibration source: no UDP socket or follower controller."""
    decoder = KerDeviceInput()
    last_data = time.monotonic()
    next_output = last_data
    while running():
        frame = stream.recv()
        now = time.monotonic()
        if frame is not None:
            frame = decoder.accept(frame)
            if frame is not None:
                raw = frame["angles"]
                if len(raw) != 16 or any(not math.isfinite(float(v)) for v in raw):
                    raise ValueError("KER 必须提供十六个有限编码器角度")
                last_data = now
                if now >= next_output:
                    record = {**frame, "pc_monotonic":now}
                    payload = json.dumps(record)
                    if output is not None:
                        output.write(payload+"\n")
                        output.flush()
                    print("[ker-inspect] " + payload, flush=True)
                    next_output = now+.05
        if now-last_data > 1:
            raise RuntimeError("KER 超过 1 秒无新鲜数据，不能进行扳机标定")
        time.sleep(.002)


def make_packet(sequence: int, stamp: float, target: list[float], *,
                left_pressed: bool = False, right_pressed: bool = False) -> bytes:
    return (f"KER2,{sequence},{stamp:.9f},{int(left_pressed)},{int(right_pressed)}," +
            ",".join(f"{q:.9f}" for q in target)).encode("ascii")


def log_sample(raw, target: list[float]) -> None:
    sample = {"raw_degrees": list(raw), "left_target_rad": target[:8], "right_target_rad": target[8:]}
    print("[ker-bridge] first_sample=" + json.dumps(sample, ensure_ascii=False), flush=True)


def configure_usb_packet_reads(stream, transport: str) -> None:
    """Avoid large bulk transfers delaying frames or losing partial timeout data.

    SDK 0.3.0 asks for 4096 bytes per streaming read. Read one USB endpoint
    packet instead; its existing buffer still assembles full KER protocol frames.
    Apply before connect(), covering handshake and the background read thread.
    """
    if transport != "usb":
        return
    read_raw = stream._read_raw

    def read_packet(size):
        endpoint = getattr(stream, "_ep_in", None)
        packet_size = getattr(endpoint, "wMaxPacketSize", 64)
        if type(packet_size) is not int or not 1 <= packet_size <= 1024:
            packet_size = 64
        return read_raw(min(size, packet_size))

    stream._read_raw = read_packet


def connect_stream(stream, transport: str) -> None:
    try:
        stream.connect()
    except OSError as error:
        if transport == "usb" and (error.errno in (errno.EACCES, errno.EPERM)
                                    or getattr(error, "backend_error_code", None) == -3):
            raise RuntimeError(
                "KER USB 权限不足（303a:4002）。在项目目录执行："
                "sudo bash scripts/configure_ker_usb.sh；完成后重新启动 KER 仿真，"
                "必要时重新插拔 USB。无需以 root 运行控制台。"
            ) from error
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/ker.yaml")
    parser.add_argument("--udp-port", type=int, default=47932)
    parser.add_argument("--transport", choices=("usb", "serial"), default="usb")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--real", action="store_true", help="要求配置已完成 V1 标定")
    parser.add_argument("--check", action="store_true", help="仅检查映射配置")
    parser.add_argument("--probe", action="store_true", help="仅读取 KER 元数据并关闭连接，不发送动作")
    parser.add_argument("--inspect", action="store_true", help="输出新鲜编码器读数用于标定，不连接从臂")
    parser.add_argument("--inspect-log", type=Path,
                        default=Path(tempfile.gettempdir()) / "openarm_ker_inspect.jsonl",
                        help="只读检测日志（含元数据、原始通道及设备时间戳）")
    parser.add_argument("--demo", action="store_true", help="仿真演示输入，不连接 KER")
    args = parser.parse_args()
    if not 1 <= args.udp_port <= 65535:
        parser.error("UDP 端口必须在 1～65535 之间")
    if args.demo and args.real:
        parser.error("演示输入仅允许仿真")
    if args.demo and args.probe:
        parser.error("设备检查不能使用演示输入")
    if args.inspect and (args.demo or args.real or args.probe or args.check):
        parser.error("--inspect 不能与演示、真机、设备检查或配置检查同时使用")
    running = True
    def stop(_sig, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    stream = None
    try:
        mapping = load_mapping(args.config, real=args.real)
        if args.check:
            print("[ker-bridge] mapping OK", flush=True)
            return 0
        if not args.demo:
            if sys.version_info < (3, 11):
                raise RuntimeError("官方 KER SDK 需要 Python 3.11+；请运行 bash scripts/install_ker.sh 创建独立环境")
            try:
                from openarm_ker.ker_stream import KERStream, CMD_STREAM
            except ImportError as error:
                raise RuntimeError("请安装 KER 依赖：bash scripts/install_ker.sh") from error
            stream = KERStream(transport=args.transport, port=args.serial_port, auto_resume=False)
            configure_usb_packet_reads(stream, args.transport)
            connect_stream(stream, args.transport)
            if args.transport == "usb":
                print("[ker-bridge] USB read mode: endpoint-sized packets (low latency)", flush=True)
            print("[ker-bridge] metadata=" + json.dumps(stream.metadata, ensure_ascii=False), flush=True)
            if args.probe:
                print("[ker-bridge] device access OK", flush=True)
                return 0
            stream.send_command(CMD_STREAM)
            if args.inspect:
                with args.inspect_log.open("w", encoding="utf-8") as output:
                    output.write(json.dumps({"event":"connected", "metadata":stream.metadata,
                                             "schema":stream._fields})+"\n")
                    output.flush()
                    print(f"[ker-bridge] inspect log: {args.inspect_log}", flush=True)
                    try:
                        inspect_stream(stream, lambda: running, output=output)
                    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
                        output.write(json.dumps({"event":"error", "message":str(error)}, ensure_ascii=False)+"\n")
                        output.flush()
                        raise
                return 0
        print("[ker-bridge] streaming on loopback UDP " + str(args.udp_port), flush=True)
        print("[ker-bridge] 同时压下左右扳机并保持 2 秒，才会使能从臂并接入首帧。", flush=True)
        start = time.monotonic()
        last_data = start
        sequence = 0
        sample_logged = False
        device_input = KerDeviceInput()
        next_status = start
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            while running:
                now = time.monotonic()
                if args.demo:
                    wave = .04*(1-math.cos(now-start))
                    target = [wave, 0, 0, 1.570796, 0, 0, 0, -.98]*2
                    left_pressed = right_pressed = True
                else:
                    data = None
                    left_pressed = right_pressed = True
                    # Bounded drain: preserve releases, reject faults, and never turn
                    # duplicate device frames into fresh PC watchdog heartbeats.
                    for _ in range(32):
                        candidate = stream.recv()
                        if candidate is None:
                            break
                        candidate = device_input.accept(candidate)
                        if candidate is None:
                            continue
                        map_angles(candidate["angles"], mapping)
                        left_pressed = left_pressed and trigger_pressed(candidate["angles"], mapping, "left")
                        right_pressed = right_pressed and trigger_pressed(candidate["angles"], mapping, "right")
                        data = candidate
                    if data is None:
                        if now-last_data > 1:
                            raise RuntimeError(f"KER 超过 1 秒无新数据：重复设备帧={device_input.duplicate_frames}，"
                                               f"最后设备时间戳={device_input.timestamp}；请检查连接后重新启动")
                        time.sleep(.002)
                        continue
                    target = map_angles(data["angles"], mapping)
                    if not sample_logged:
                        log_sample(data["angles"], target)
                        print("[ker-bridge] device_frame=" + json.dumps(
                            {"fields":sorted(data), "seq":data.get("seq"), "timestamp":data.get("timestamp"),
                             "error_mask":data.get("error_mask"), "errors":data.get("errors")}), flush=True)
                        sample_logged = True
                if now >= next_status:
                    if args.demo:
                        detail = "demo"
                    else:
                        parts = []
                        for side in ("left", "right"):
                            arm = mapping[side]
                            raw_angle = float(data["angles"][arm["gripper_channel"]])
                            ratio = (raw_angle-arm["gripper_open_degrees"])/(arm["gripper_closed_degrees"]-arm["gripper_open_degrees"])
                            parts.append(f"{side}={raw_angle:.1f}deg/{ratio*100:.0f}%")
                        detail = " ".join(parts) + f" device_seq={data.get('seq', 'unavailable')} device_timestamp={data.get('timestamp', 'unavailable')}"
                    print(f"[ker-bridge] triggers {detail} pressed=({int(left_pressed)},{int(right_pressed)}) "
                          "threshold=90% (both required for 2s)", flush=True)
                    next_status = now+1.0
                last_data = now
                sender.sendto(make_packet(sequence, now, target, left_pressed=left_pressed,
                                          right_pressed=right_pressed), ("127.0.0.1", args.udp_port))
                sequence += 1
                time.sleep(.005)
        return 0
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        print(f"[ker-bridge][ERROR] {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        if stream is not None:
            stream.close()


if __name__ == "__main__":
    raise SystemExit(main())

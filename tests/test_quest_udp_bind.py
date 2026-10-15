"""Default Quest UDP bind is loopback, and a datagram sent there is received."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "quest"))

RECEIVERS = (
    "quest_pose_receiver.cpp",
)


def compile_smoke() -> Path:
    binary = Path(tempfile.mkdtemp(prefix="quest-udp-bind-")) / "quest_udp_bind_smoke"
    source = ROOT / "tests" / "quest_udp_bind_smoke.cpp"
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
            "-Isrc",
            str(source),
            "-o",
            str(binary),
        ],
        cwd=ROOT,
        check=True,
    )
    return binary


class QuestUdpBindTests(unittest.TestCase):
    def setUp(self) -> None:
        self._saved = os.environ.get("OPENARM_QUEST_UDP_HOST")
        os.environ.pop("OPENARM_QUEST_UDP_HOST", None)

    def tearDown(self) -> None:
        if self._saved is None:
            os.environ.pop("OPENARM_QUEST_UDP_HOST", None)
        else:
            os.environ["OPENARM_QUEST_UDP_HOST"] = self._saved

    def test_receivers_bind_through_shared_helper(self) -> None:
        for name in RECEIVERS:
            text = (ROOT / "src" / "io" / name).read_text(encoding="utf-8")
            self.assertIn("bind_quest_udp(", text)
            self.assertNotIn("INADDR_ANY", text)

    def test_python_target_defaults_to_loopback_and_can_be_overridden(self) -> None:
        from quest_bridge import QuestBridge, quest_udp_host, resolve_udp_host

        self.assertEqual(quest_udp_host(), "127.0.0.1")
        self.assertEqual(resolve_udp_host(None), "127.0.0.1")
        self.assertEqual(resolve_udp_host("  "), "127.0.0.1")
        self.assertEqual(resolve_udp_host("192.0.2.10"), "192.0.2.10")

        os.environ["OPENARM_QUEST_UDP_HOST"] = "192.0.2.9"
        self.assertEqual(quest_udp_host(), "192.0.2.9")
        self.assertEqual(resolve_udp_host(None), "192.0.2.9")
        self.assertEqual(resolve_udp_host("10.1.2.3"), "10.1.2.3")

        with patch("quest_bridge.create_self_signed_cert", return_value=None):
            bridge = QuestBridge()
            explicit = QuestBridge(udp_host="10.9.8.7")
        self.assertEqual(bridge.host, "0.0.0.0")
        self.assertEqual(bridge.right_forwarder.target, ("192.0.2.9", 47911))
        self.assertEqual(bridge.left_forwarder.target, ("192.0.2.9", 47912))
        self.assertEqual(explicit.right_forwarder.target[0], "10.9.8.7")
        bridge.right_forwarder.sock.close()
        bridge.left_forwarder.sock.close()
        explicit.right_forwarder.sock.close()
        explicit.left_forwarder.sock.close()

    def test_capture_shortcut_follows_same_host(self) -> None:
        sys.path.insert(0, str(ROOT / "script"))
        from openarm_app.services import quest_udp_host as console_host
        from quest_bridge import capture_udp_target, quest_udp_host

        self.assertEqual(capture_udp_target(), ("127.0.0.1", 47934))
        self.assertEqual(console_host(), "127.0.0.1")
        os.environ["OPENARM_QUEST_UDP_HOST"] = "192.0.2.9"
        os.environ["OPENARM_QUEST_CAPTURE_PORT"] = "47999"
        try:
            self.assertEqual(quest_udp_host(), "192.0.2.9")
            self.assertEqual(console_host(), "192.0.2.9")
            self.assertEqual(capture_udp_target(), ("192.0.2.9", 47999))
        finally:
            os.environ.pop("OPENARM_QUEST_CAPTURE_PORT", None)

    def test_webpage_ports_stay_on_all_interfaces(self) -> None:
        from quest_bridge import QuestBridge

        with patch("quest_bridge.create_self_signed_cert", return_value=None):
            bridge = QuestBridge()
        self.assertEqual(bridge.host, "0.0.0.0")
        self.assertEqual(bridge.http_port, 8443)
        self.assertEqual(bridge.ws_port, 8442)
        bridge.right_forwarder.sock.close()
        bridge.left_forwarder.sock.close()

    def test_bridge_forwarder_delivers_to_loopback_bind(self) -> None:
        from quest_bridge import HandBridgeStats, QuestUdpForwarder

        binary = compile_smoke()
        port = self._free_port()
        env = os.environ.copy()
        env.pop("OPENARM_QUEST_UDP_HOST", None)
        proc = subprocess.Popen(
            [str(binary), str(port), "--recv"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )
        forwarder = QuestUdpForwarder("127.0.0.1", port, HandBridgeStats("right"))
        try:
            line = proc.stdout.readline().strip()
            self.assertEqual(line, f"bound 127.0.0.1:{port}")
            forwarder.send_controller(
                {
                    "timestamp": 42,
                    "gripActive": True,
                    "trigger": 0.5,
                    "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "buttonClose": False,
                    "buttonOpen": True,
                }
            )
            rest, err = proc.communicate(timeout=3)
            self.assertEqual(proc.returncode, 0, err)
            self.assertTrue(rest.strip().startswith("recv 42,1,0.500000,0.100000,0.200000,0.300000"))
            self.assertTrue(rest.strip().endswith(",0,1"))
        finally:
            forwarder.sock.close()
            if proc.poll() is None:
                proc.kill()
                proc.communicate(timeout=3)

    def test_default_socket_is_loopback_and_receives_there(self) -> None:
        binary = compile_smoke()
        port = self._free_port()
        env = os.environ.copy()
        env.pop("OPENARM_QUEST_UDP_HOST", None)
        proc = subprocess.Popen(
            [str(binary), str(port), "--recv"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )
        try:
            line = proc.stdout.readline().strip()
            self.assertEqual(line, f"bound 127.0.0.1:{port}")
            self._assert_netstat(f"127.0.0.1:{port}")
            payload = "1700000000000,1,0.250000,0.100000,0.200000,0.300000,0,0,0,1,0,0"
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.sendto(b"from-other", ("127.0.0.2", port))
                sock.sendto(payload.encode("ascii"), ("127.0.0.1", port))
            finally:
                sock.close()
            rest, err = proc.communicate(timeout=3)
            self.assertEqual(proc.returncode, 0, err)
            self.assertEqual(rest.strip(), "recv " + payload)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.communicate(timeout=3)

    def test_open_bind_receives_non_loopback_destination(self) -> None:
        binary = compile_smoke()
        port = self._free_port()
        env = os.environ.copy()
        env["OPENARM_QUEST_UDP_HOST"] = "0.0.0.0"
        proc = subprocess.Popen(
            [str(binary), str(port), "--recv"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )
        try:
            line = proc.stdout.readline().strip()
            self.assertEqual(line, f"bound 0.0.0.0:{port}")
            self._assert_netstat(f"0.0.0.0:{port}")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.sendto(b"lan", ("127.0.0.2", port))
            finally:
                sock.close()
            rest, err = proc.communicate(timeout=3)
            self.assertEqual(proc.returncode, 0, err)
            self.assertEqual(rest.strip(), "recv lan")
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.communicate(timeout=3)

    def test_invalid_host_fails_closed(self) -> None:
        binary = compile_smoke()
        env = os.environ.copy()
        env["OPENARM_QUEST_UDP_HOST"] = "not-an-ip"
        result = subprocess.run(
            [str(binary), "0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("bind failed", result.stderr)

    def _free_port(self) -> int:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()
        return port

    def _assert_netstat(self, local: str) -> None:
        if shutil.which("netstat") is None:
            self.skipTest("netstat is not installed")
        listing = subprocess.run(
            ["netstat", "-uln"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        ).stdout
        self.assertIn(local, listing)


if __name__ == "__main__":
    unittest.main()

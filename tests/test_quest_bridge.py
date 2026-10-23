import asyncio
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest"))


class QuestBridgeTests(unittest.TestCase):
    def test_bridge_disconnect_clears_client_without_exception_lookup_error(self):
        from quest_bridge import QuestBridge
        from websockets.exceptions import ConnectionClosed

        class Connection:
            remote_address = ('test', 0)

            def __aiter__(self):
                async def iterate():
                    raise ConnectionClosed(None, None)
                    yield  # This is an async iterator, even when it closes immediately.
                return iterate()

        bridge = QuestBridge.__new__(QuestBridge)
        bridge.ws_clients = set()
        bridge.client_connected = False
        asyncio.run(bridge.handle_client(Connection()))
        self.assertFalse(bridge.client_connected)
        self.assertEqual(bridge.ws_clients, set())

    def test_newer_pose_is_forwarded_and_older_pose_is_dropped(self):
        from quest_bridge import _ClientSession

        class Forwarder:
            def __init__(self):
                self.sent = []

            def send_controller(self, controller):
                self.sent.append(controller)

        class Stats:
            def maybe_report(self, _connected):
                return None

        class Bridge:
            bimanual = True
            client_connected = True
            right_forwarder = Forwarder()
            left_forwarder = Forwarder()
            right_stats = Stats()
            left_stats = Stats()

        def pose(timestamp, hand):
            return {
                "timestamp": timestamp,
                hand: {
                    "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "quaternion": {"x": 0, "y": 0, "z": 0, "w": 1},
                    "gripActive": True,
                    "trigger": 0,
                },
            }

        async def run():
            session = _ClientSession(Bridge(), None)
            await session.ingest_pose(pose(10, "rightController"))
            await session.ingest_pose(pose(9, "rightController"))
            await session.ingest_pose({"type": "webrtc_offer", "sdp": ""})
            await session.handle_message(pose(11, "leftController"))

        asyncio.run(run())
        self.assertEqual([item["timestamp"] for item in Bridge.right_forwarder.sent], [10])
        self.assertEqual([item["timestamp"] for item in Bridge.left_forwarder.sent], [11])

    def test_webrtc_pose_channel_reaches_the_udp_forwarder(self):
        try:
            from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
        except ImportError as error:
            self.skipTest(str(error))
        from quest_bridge import _ClientSession

        class Forwarder:
            def __init__(self):
                self.sent = []

            def send_controller(self, controller):
                self.sent.append(controller)

        class Stats:
            def maybe_report(self, _connected):
                return None

        class Socket:
            def __init__(self):
                self.sent = []

            async def send(self, text):
                self.sent.append(text)

        class Bridge:
            bimanual = False
            client_connected = True
            right_forwarder = Forwarder()
            left_forwarder = Forwarder()
            right_stats = Stats()
            left_stats = Stats()

        async def run():
            bridge = Bridge()
            socket = Socket()
            session = _ClientSession(bridge, socket)
            offer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
            channel = offer.createDataChannel("poses", ordered=False, maxRetransmits=0)
            opened = asyncio.Event()
            channel.on("open", opened.set)
            await offer.setLocalDescription(await offer.createOffer())
            deadline = asyncio.get_running_loop().time() + 1.5
            while offer.iceGatheringState != "complete" and asyncio.get_running_loop().time() < deadline:
                await asyncio.sleep(0.02)
            await session.accept_webrtc({"type": "webrtc_offer", "sdp": offer.localDescription.sdp})
            answer = json.loads(socket.sent[-1])
            await offer.setRemoteDescription(RTCSessionDescription(answer["sdp"], "answer"))
            await asyncio.wait_for(opened.wait(), 3)
            channel.send(json.dumps({
                "timestamp": 42,
                "rightController": {
                    "position": {"x": 0.2, "y": 0.0, "z": 0.4},
                    "quaternion": {"x": 0, "y": 0, "z": 0, "w": 1},
                    "gripActive": False,
                    "trigger": 0.25,
                },
            }))
            for _ in range(40):
                if bridge.right_forwarder.sent:
                    break
                await asyncio.sleep(0.05)
            await offer.close()
            await session.close()
            return bridge.right_forwarder.sent

        sent = asyncio.run(run())
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0]["timestamp"], 42)
        self.assertAlmostEqual(sent[0]["trigger"], 0.25)


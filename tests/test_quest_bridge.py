import asyncio
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


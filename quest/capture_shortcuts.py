"""Recognize thumbstick clicks without repeating held or interrupted presses."""
from __future__ import annotations


class CaptureShortcuts:
    def __init__(self) -> None:
        self.hands: dict[str, tuple[float, float | None, bool]] = {}

    def update(self, hand: str, controller: dict | None, now: float) -> str | None:
        if not controller or "stickPressed" not in controller:
            self.hands.pop(hand, None)
            return None
        pressed = controller["stickPressed"] is True
        previous = self.hands.get(hand)
        if previous is None or now - previous[0] > .25:
            # Require a released sample before arming after startup/tracking loss.
            self.hands[hand] = (now, None, not pressed)
            return None
        _, started, armed = previous
        if pressed and armed and started is None:
            started = now
        action = None
        if not pressed:
            if armed and started is not None:
                duration = now - started
                if duration >= .05:
                    action = f"{hand}-{'long' if duration >= 1.0 else 'short'}"
            started, armed = None, True
        self.hands[hand] = (now, started, armed)
        return action

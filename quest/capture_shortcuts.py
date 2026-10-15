"""Recognize thumbstick clicks.

A short click is a press released quickly. A long press fires while the stick
is still held, so the headset can show haptic progress and the action does not
wait for release. Letting go after the short window but before the long
threshold cancels the press.
"""
from __future__ import annotations

# Hold this long and the long action fires immediately.
LONG_PRESS_S = 1.5
# Released presses in this window are short clicks. Longer holds that never
# reach LONG_PRESS_S are cancelled.
SHORT_PRESS_S = 0.05
SHORT_MAX_S = 0.40
# A gap this long means tracking dropped; the next sample must show a release.
ARM_GAP_S = 0.25


class CaptureShortcuts:
    def __init__(self) -> None:
        # last sample time, press start, armed, long action already emitted
        self.hands: dict[str, tuple[float, float | None, bool, bool]] = {}

    def update(self, hand: str, controller: dict | None, now: float) -> str | None:
        if not controller or "stickPressed" not in controller:
            self.hands.pop(hand, None)
            return None
        pressed = controller["stickPressed"] is True
        previous = self.hands.get(hand)
        if previous is None or now - previous[0] > ARM_GAP_S:
            # Require a released sample before arming after startup/tracking loss.
            self.hands[hand] = (now, None, not pressed, False)
            return None
        _, started, armed, fired = previous
        action = None
        if pressed:
            if armed and started is None:
                started = now
            if (
                armed
                and started is not None
                and not fired
                and now - started >= LONG_PRESS_S
            ):
                action = f"{hand}-long"
                fired = True
        else:
            if armed and started is not None and not fired:
                duration = now - started
                if SHORT_PRESS_S <= duration < SHORT_MAX_S:
                    action = f"{hand}-short"
            started, armed, fired = None, True, False
        self.hands[hand] = (now, started, armed, fired)
        return action

"""Load safety thresholds from tests/safety_contract.hpp.

That header is the only copy of the numbers. Do not restate them here.
"""

from __future__ import annotations

import re
from pathlib import Path

_HEADER = Path(__file__).with_name("safety_contract.hpp").read_text(encoding="utf-8")


def _value(name: str) -> float:
    match = re.search(rf"\b{name}\s*=\s*([0-9]+(?:\.[0-9]+)?)", _HEADER)
    if match is None:
        raise RuntimeError(f"{name} is missing from safety_contract.hpp")
    return float(match.group(1))


QUEST_TIMEOUT_SEC = _value("kQuestTimeoutSec")
SOLVE_FAIL_FREEZE_COUNT = int(_value("kSolveFailFreezeCount"))
CAN_FEEDBACK_MAX_AGE_MS = _value("kCanFeedbackMaxAgeMs")
CAN_ERROR_CODE_MAX_OK = int(_value("kCanErrorCodeMaxOk"))
ELBOW_JOINT_INDEX = int(_value("kElbowJointIndex"))
ELBOW_URDF_UPPER_RAD = _value("kElbowUrdfUpperRad")

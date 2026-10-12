"""Translate WebXR SEW points into the existing skeleton_proximal stream."""
from __future__ import annotations

import math


def arm_fields(arm: object) -> list[float]:
    invalid = [0.0]*9
    if not isinstance(arm, dict):
        return invalid
    try:
        points = [arm[key] for key in ('shoulder', 'elbow', 'wrist')]
        if any(not isinstance(p, list) or len(p) != 3 for p in points):
            return invalid
        if any(type(value) not in (int, float) or not math.isfinite(value)
               for point in points for value in point):
            return invalid
        directions, lengths = [], []
        for start, end in zip(points, points[1:]):
            delta = [b-a for a, b in zip(start, end)]
            length = math.sqrt(sum(v*v for v in delta))
            if not .08 <= length <= .65:
                return invalid
            # Right-handed 180-degree X rotation: Quest +Y up/-Z forward
            # -> legacy skeleton coordinates +Y down/+Z forward.
            directions.extend([delta[0]/length, -delta[1]/length, -delta[2]/length])
            lengths.append(length)
        return directions + lengths + [1.0]
    except (KeyError, TypeError, ValueError, OverflowError):
        return invalid


def skeleton_packet(body: object, timestamp: object) -> bytes:
    left, right = [0.0]*9, [0.0]*9
    if (isinstance(body, dict) and body.get('source') == 'webxr'
            and body.get('timestamp') == timestamp):
        left, right = arm_fields(body.get('left')), arm_fields(body.get('right'))
    valid = int(left[-1] > 0 or right[-1] > 0)
    # The C++ receiver records freshness using the PC monotonic clock.
    values = [int(timestamp)//1000, valid, *left, *right]
    return ','.join(str(value) for value in values).encode('ascii')


def skeleton_diagnostics(body: object, timestamp: object) -> tuple[str, str]:
    """Explain invalid frames without trusting client-provided free-form log text."""
    if not isinstance(body, dict):
        return ('missing_body_payload',)*2
    if body.get('source') != 'webxr':
        return ('wrong_source',)*2
    if body.get('timestamp') != timestamp:
        return ('timestamp_mismatch',)*2
    diagnostics = body.get('diagnostics')
    diagnostics = diagnostics if isinstance(diagnostics, dict) else {}
    allowed = {'no_body', 'missing_joint', 'unavailable_pose', 'emulated_position',
               'invalid_position', 'pose_error'}
    result = []
    for side in ('left', 'right'):
        arm = body.get(side)
        if arm_fields(arm)[-1] > 0:
            result.append('valid')
        elif isinstance(arm, dict):
            result.append('invalid_geometry')
        else:
            reason = diagnostics.get(side)
            result.append(reason if isinstance(reason, str) and reason in allowed else 'missing_arm')
    return tuple(result)

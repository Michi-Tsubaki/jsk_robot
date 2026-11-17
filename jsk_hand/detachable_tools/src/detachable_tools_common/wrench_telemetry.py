"""Portable six-axis sensor contract: SI units, frame, provenance and freshness.

Independent of tasks, robots, ROS and simulators. This is a measured/reconstructed
wrench, never a six-axis interpretation of a scalar jaw force.
"""

import math


def vector(value, size):
    if value is None or len(value) != size:
        raise ValueError(f"Expected {size} finite values")
    out = [float(x) for x in value]
    if not all(math.isfinite(x) for x in out):
        raise ValueError("Nonfinite vector")
    return out


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def transform_wrench(force, torque, rotation, translation):
    """T_target_source: rotate axes AND shift the moment reference origin."""
    rotation = vector(rotation, 9)
    translation = vector(translation, 3)

    def rotate(v):
        return [sum(rotation[3 * i + j] * v[j] for j in range(3)) for i in range(3)]

    force = rotate(vector(force, 3))
    torque = rotate(vector(torque, 3))
    offset = cross(translation, force)
    return force, [t + d for t, d in zip(torque, offset)]


def wrench_state(
    *,
    side,
    frame_id,
    stamp_s,
    force_n=None,
    torque_nm=None,
    source="unavailable",
    calibrated=False,
    reason=None,
):
    result = {
        "schema_version": 1,
        "kind": "wrist_wrench",
        "side": side,
        "frame_id": frame_id,
        "stamp_s": stamp_s if math.isfinite(stamp_s) else None,
        "force_n": None,
        "torque_nm": None,
        "source": source,
        "calibrated": bool(calibrated),
        "valid": False,
        "reason": reason,
        "convention": "arm_on_distal_assembly",
        "gravity_compensated": False,
    }
    if reason:
        return result
    try:
        if not frame_id or not math.isfinite(stamp_s) or stamp_s < 0:
            raise ValueError("Missing frame or timestamp")
        result["force_n"] = vector(force_n, 3)
        result["torque_nm"] = vector(torque_nm, 3)
    except (TypeError, ValueError):
        result.update(force_n=None, torque_nm=None, reason="invalid_wrench")
        return result
    result.update(valid=True, reason="ok")
    return result


class WrenchStream:
    """Driver-independent conversion of raw WrenchStamped-like observations."""

    def __init__(self, side, config, timeout_s=0.25):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.side, self.config, self.timeout_s = side, config, timeout_s
        self.bias = vector(config.get("bias", [0] * 6), 6)
        self.rotation = vector(config.get("rotation", [1, 0, 0, 0, 1, 0, 0, 0, 1]), 9)
        # Do not silently accept scaling/shearing as a frame rotation.
        columns = [[self.rotation[3 * i + j] for i in range(3)] for j in range(3)]
        if (
            any(
                abs(sum(a * b for a, b in zip(columns[i], columns[j])) - (i == j))
                > 1e-6
                for i in range(3)
                for j in range(3)
            )
            or sum(a * b for a, b in zip(cross(columns[0], columns[1]), columns[2]))
            < 0.999999
        ):
            raise ValueError("rotation must be a proper orthonormal matrix")
        self.translation = vector(config.get("translation_m", [0, 0, 0]), 3)
        self.sign = config.get("sign", 1)
        if self.sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")
        self.previous = None

    def invalid(self, stamp_s, reason):
        return wrench_state(
            side=self.side,
            frame_id=self.config["frame_id"],
            stamp_s=stamp_s,
            source=self.config["source"],
            reason=reason,
        )

    def update(self, *, stamp_s, now_s, frame_id, force_n, torque_nm):
        if not all(math.isfinite(v) for v in (stamp_s, now_s)) or stamp_s < 0:
            return self.invalid(stamp_s, "invalid_timestamp")
        if not 0 <= now_s - stamp_s <= self.timeout_s:
            return self.invalid(stamp_s, "stale_or_future_sample")
        if self.previous is not None and stamp_s <= self.previous:
            return self.invalid(stamp_s, "nonmonotone_timestamp")
        if frame_id != self.config["input_frame"]:
            return self.invalid(stamp_s, "unexpected_frame")
        try:
            raw = vector(force_n, 3) + vector(torque_nm, 3)
            corrected = [self.sign * (a - b) for a, b in zip(raw, self.bias)]
            force, torque = transform_wrench(
                corrected[:3], corrected[3:], self.rotation, self.translation
            )
        except (ValueError, TypeError):
            return self.invalid(stamp_s, "invalid_wrench")
        self.previous = stamp_s
        return wrench_state(
            side=self.side,
            frame_id=self.config["frame_id"],
            stamp_s=stamp_s,
            force_n=force,
            torque_nm=torque,
            source=self.config["source"],
            calibrated=self.config.get("calibrated", False),
        )

    def expired(self, now_s):
        if self.previous is None or not 0 <= now_s - self.previous <= self.timeout_s:
            return self.invalid(now_s, "sample_timeout")
        return None

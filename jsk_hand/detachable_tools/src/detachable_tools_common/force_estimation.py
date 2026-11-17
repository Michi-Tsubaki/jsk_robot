"""Portable tool-force contract shared verbatim with detachable_tools.

No simulator, ROS, task, anatomy, timing schedule or tissue model dependency.
Input effort is torque at the drive coordinate, after the internal gearbox.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import asdict, dataclass

SCHEMA_VERSION = 1


@dataclass(frozen=True)
class ToolCalibration:
    torque_constant_nm_per_a: float = 1.15
    current_bias_a: float = 0.002
    current_resolution_a: float = 0.001
    current_sensor_limit_a: float = 0.8
    coulomb_friction_nm: float = 0.002
    viscous_friction_nms_per_rad: float = 0.004
    friction_transition_rad_s: float = 0.01
    reflected_inertia_kg_m2: float = 0.00002
    minimum_jacobian_m_per_rad: float = 0.0001

    def __post_init__(self):
        for name, value in asdict(self).items():
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name} must be finite numeric")
            if name != "current_bias_a" and value < 0:
                raise ValueError(f"{name} must be nonnegative")
        for name in (
            "torque_constant_nm_per_a",
            "current_resolution_a",
            "current_sensor_limit_a",
            "friction_transition_rad_s",
            "minimum_jacobian_m_per_rad",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")


def motor_effort(current_a, torque_constant_nm_per_a, current_bias_a=0.0):
    return torque_constant_nm_per_a * (current_a - current_bias_a)


def force_from_effort(effort_nm, compensation_nm, jacobian_m_per_rad, minimum_jacobian):
    if not math.isfinite(minimum_jacobian) or minimum_jacobian <= 0:
        raise ValueError("minimum_jacobian must be finite and positive")
    if (
        not all(
            math.isfinite(x) for x in (effort_nm, compensation_nm, jacobian_m_per_rad)
        )
        or abs(jacobian_m_per_rad) < minimum_jacobian
    ):
        return None
    return (effort_nm - compensation_nm) / jacobian_m_per_rad


def estimate_force(
    *,
    tool,
    side,
    stamp_s,
    position_rad,
    gap_m,
    jacobian_m_per_rad,
    calibration,
    effort_nm=None,
    current_a=None,
    compensation_nm=0.0,
    compensation_complete=False,
    source="unavailable",
    calibrated=False,
    unavailable_reason=None,
):
    """Signed force conjugate to opening gap, equal/opposite at the two jaws.

    tau*dq = F*dg. There is no additional factor of two. No claim of a 6-axis
    wrench or contact detection is made. Missing data is None, never zero.
    """
    if effort_nm is not None and current_a is not None:
        raise ValueError("Supply current OR effort; do not apply Kt twice")
    result = {
        "schema_version": SCHEMA_VERSION,
        "tool": tool,
        "side": side,
        "stamp_s": stamp_s,
        "position_rad": position_rad,
        "gap_m": gap_m,
        "jacobian_m_per_rad": jacobian_m_per_rad,
        "current_a": current_a,
        "effort_nm": effort_nm,
        "compensation_nm": compensation_nm,
        "compensation_complete": bool(compensation_complete),
        "source": source,
        "calibrated": bool(calibrated),
        "valid": False,
        "reason": unavailable_reason,
        "force_n": None,
        "opening_force_n": None,
        "force_definition": "signed force conjugate to gap; magnitude per jaw, not a summed or 6-axis force",
    }
    # Telemetry stays strict JSON even when a backend supplies NaN/Inf.
    for key, value in result.items():
        if isinstance(value, float) and not math.isfinite(value):
            result[key] = None
    if unavailable_reason:
        return result
    if not all(
        x is not None and math.isfinite(x)
        for x in (stamp_s, position_rad, gap_m, jacobian_m_per_rad, compensation_nm)
    ):
        result["reason"] = "invalid_geometry_or_compensation"
        return result
    if current_a is not None:
        if (
            not math.isfinite(current_a)
            or abs(current_a) >= calibration.current_sensor_limit_a
        ):
            result["reason"] = "invalid_or_saturated_current"
            return result
        effort_nm = motor_effort(
            current_a, calibration.torque_constant_nm_per_a, calibration.current_bias_a
        )
        result["effort_nm"] = effort_nm
    if effort_nm is None or not math.isfinite(effort_nm):
        result["reason"] = "effort_unavailable"
        return result
    force = force_from_effort(
        effort_nm,
        compensation_nm,
        jacobian_m_per_rad,
        calibration.minimum_jacobian_m_per_rad,
    )
    if force is None:
        result["reason"] = "singular_transmission"
        return result
    result.update(
        valid=True, reason="ok", force_n=force, opening_force_n=max(force, 0.0)
    )
    return result


def interpolate_profile(profile, side, position_rad):
    """Tool-specific CAD/calibration table; independent of the calling task."""
    table = profile["sides"][side]
    q = table["position_rad"]
    if (
        not math.isfinite(position_rad)
        or position_rad < q[0] - 1e-10
        or position_rad > q[-1] + 1e-10
    ):
        raise ValueError("position_outside_profile")
    i = min(max(bisect.bisect_right(q, position_rad) - 1, 0), len(q) - 2)
    weight = min(max((position_rad - q[i]) / (q[i + 1] - q[i]), 0.0), 1.0)

    def blend(a, b):
        return (
            [(1 - weight) * x + weight * y for x, y in zip(a, b)]
            if isinstance(a, list)
            else (1 - weight) * a + weight * b
        )

    return {
        key: blend(table[key][i], table[key][i + 1])
        for key in ("gap_m", "jacobian_m_per_rad", "mass_moment_derivative_kg_m")
    }


def mechanism_compensation(
    calibration,
    velocity_rad_s,
    acceleration_rad_s2=0.0,
    mass_moment_derivative_kg_m=None,
    gravity_base_m_s2=None,
):
    """Joint-side friction/inertia and optional gravity in the tool base frame."""
    friction = calibration.coulomb_friction_nm * math.tanh(
        velocity_rad_s / calibration.friction_transition_rad_s
    )
    friction += calibration.viscous_friction_nms_per_rad * velocity_rad_s
    inertia = calibration.reflected_inertia_kg_m2 * acceleration_rad_s2
    gravity = 0.0
    if gravity_base_m_s2 is not None and mass_moment_derivative_kg_m is not None:
        gravity = -sum(
            a * b for a, b in zip(mass_moment_derivative_kg_m, gravity_base_m_s2)
        )
    return friction + inertia + gravity


class ToolForceStream:
    """Middleware-independent attachment, timestamp and missing-signal handling."""

    def __init__(self, profiles, timeout_s=0.25):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.profiles, self.timeout_s = profiles, timeout_s
        self.tool, self.side, self.attached, self.joint, self.frame = (
            "",
            "right",
            False,
            "",
            "",
        )
        self.previous = None
        self.last_state = None

    def select(self, tool, side, attached, joint, frame=""):
        selected = (tool, side, attached, joint, frame)
        if selected != (self.tool, self.side, self.attached, self.joint, self.frame):
            self.tool, self.side, self.attached, self.joint, self.frame = selected
            self.previous = self.last_state = None
            return True
        return False

    def invalid(self, stamp_s, reason, source="unavailable"):
        return estimate_force(
            tool=self.tool,
            side=self.side,
            stamp_s=stamp_s,
            position_rad=None,
            gap_m=None,
            jacobian_m_per_rad=None,
            calibration=ToolCalibration(),
            source=source,
            unavailable_reason=reason,
        )

    def update(
        self,
        *,
        stamp_s,
        now_s,
        position_rad,
        velocity_rad_s=None,
        effort_nm=None,
        current_a=None,
        source="joint_effort",
        gravity_base_m_s2=None,
    ):
        if not self.attached:
            return self.invalid(stamp_s, "tool_detached", source)
        profile = self.profiles.get(self.tool)
        if profile is None or self.side not in profile["sides"]:
            return self.invalid(stamp_s, "unsupported_tool", source)
        if not all(math.isfinite(x) for x in (stamp_s, now_s)) or stamp_s < 0:
            return self.invalid(stamp_s, "invalid_timestamp", source)
        if now_s - stamp_s > self.timeout_s or stamp_s > now_s + self.timeout_s:
            return self.invalid(stamp_s, "stale_or_future_sample", source)
        if self.previous is not None and stamp_s <= self.previous[0]:
            return self.invalid(stamp_s, "nonmonotone_timestamp", source)
        if position_rad is None or not math.isfinite(position_rad):
            return self.invalid(stamp_s, "position_unavailable", source)
        try:
            geometry = interpolate_profile(profile, self.side, position_rad)
        except ValueError as exc:
            return self.invalid(stamp_s, str(exc), source)
        config = ToolCalibration(**profile["calibration"])
        previous = self.previous
        if previous is not None and stamp_s - previous[0] > self.timeout_s:
            previous = None  # Re-establish velocity/acceleration after a data gap.
        if velocity_rad_s is None:
            velocity_rad_s = (
                0.0
                if previous is None
                else (position_rad - previous[1]) / (stamp_s - previous[0])
            )
        if not math.isfinite(velocity_rad_s):
            return self.invalid(stamp_s, "invalid_velocity", source)
        acceleration = (
            0.0
            if previous is None
            else (velocity_rad_s - previous[2]) / (stamp_s - previous[0])
        )
        if gravity_base_m_s2 is not None and (
            len(gravity_base_m_s2) != 3
            or not all(math.isfinite(v) for v in gravity_base_m_s2)
        ):
            return self.invalid(stamp_s, "invalid_gravity", source)
        compensation = mechanism_compensation(
            config,
            velocity_rad_s,
            acceleration,
            geometry["mass_moment_derivative_kg_m"],
            gravity_base_m_s2,
        )
        state = estimate_force(
            tool=self.tool,
            side=self.side,
            stamp_s=stamp_s,
            position_rad=position_rad,
            gap_m=geometry["gap_m"],
            jacobian_m_per_rad=geometry["jacobian_m_per_rad"],
            calibration=config,
            effort_nm=effort_nm,
            current_a=current_a,
            compensation_nm=compensation,
            compensation_complete=gravity_base_m_s2 is not None
            and previous is not None,
            source=source,
            calibrated=profile.get("calibrated", False),
        )
        state["gap_velocity_m_s"] = geometry["jacobian_m_per_rad"] * velocity_rad_s
        self.previous = (stamp_s, position_rad, velocity_rad_s)
        self.last_state = state
        return state

    def expired(self, now_s):
        if not self.attached:
            return self.invalid(now_s, "tool_detached")
        if (
            self.previous is None
            or now_s - self.previous[0] > self.timeout_s
            or now_s < self.previous[0]
        ):
            return self.invalid(now_s, "sample_timeout")
        return None

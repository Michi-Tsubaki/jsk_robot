"""Tool-owned force telemetry for ROS 1/2; subscribes to state, never commands motors."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .description import package_share_path
from .force_estimation import ToolForceStream
from .ros_compat import node_argv, ros_compat


def seconds(stamp):
    if hasattr(stamp, "to_sec"):
        return stamp.to_sec()
    return float(stamp.sec) + stamp.nanosec * 1e-9


class ToolForcePublisher:
    def __init__(
        self,
        node,
        *,
        side,
        profile_dir=None,
        timeout_s=0.25,
        input_topic="joint_states",
        output_topic="tool_force",
        input_unit="torque_nm",
    ):
        self.node, self.side, self.input_unit = node, side, input_unit
        directory = (
            Path(profile_dir)
            if profile_dir
            else package_share_path() / "config/tool_force"
        )
        profiles = {}
        for path in directory.glob("*.json"):
            data = json.loads(path.read_text())
            if data.get("schema_version") == 1 and "tool" in data:
                profiles[data["tool"]] = data
        self.stream = ToolForceStream(profiles, timeout_s)
        self.plugins = {}
        self.gravity = None
        self.force_pub = ros_compat.create_publisher(
            node, output_topic, JointState, queue_size=10
        )
        self.status_pub = ros_compat.create_publisher(
            node, output_topic + "/status", DiagnosticArray, queue_size=10
        )
        self.subscriptions = [
            ros_compat.create_subscription(
                node,
                "tool_state",
                DiagnosticArray,
                self.tool_state,
                queue_size=1,
                latch=True,
            ),
            ros_compat.create_subscription(
                node,
                "robot_description",
                String,
                self.description,
                queue_size=1,
                latch=True,
            ),
            ros_compat.create_subscription(
                node, input_topic, JointState, self.joints, queue_size=10
            ),
            ros_compat.create_subscription(
                node, "tool_gravity", Vector3Stamped, self.gravity_sample, queue_size=10
            ),
        ]
        self.timer = ros_compat.create_timer(node, min(0.1, timeout_s / 2), self.tick)

    def now(self):
        return ros_compat.stamp_now(self.node)

    def tool_state(self, msg):
        for status in msg.status:
            values = {v.key: v.value for v in status.values}
            if values.get("side") != self.side or "tool" not in values:
                continue
            changed = self.stream.select(
                values["tool"],
                "left" if self.side == "lhand" else "right",
                values.get("attached", "false").lower() == "true",
                values.get("command_joint", ""),
                values.get("root_link", ""),
            )
            if changed:
                self.gravity = None
                stamp = self.now()
                self.publish(
                    self.stream.invalid(
                        seconds(stamp),
                        "tool_changed" if self.stream.attached else "tool_detached",
                    ),
                    stamp,
                )

    def description(self, msg):
        self.plugins = {}
        try:
            for control in ET.fromstring(msg.data).findall("ros2_control"):
                plugin = control.findtext("hardware/plugin", "")
                for joint in control.findall("joint"):
                    self.plugins[joint.get("name")] = plugin
        except ET.ParseError:
            pass

    def gravity_sample(self, msg):
        # The vector must already be expressed in the selected tool base frame.
        if msg.header.frame_id == self.stream.frame:
            self.gravity = (
                seconds(msg.header.stamp),
                [msg.vector.x, msg.vector.y, msg.vector.z],
            )

    def joints(self, msg):
        now = seconds(self.now())
        stamp_s = seconds(msg.header.stamp)
        joint = self.stream.joint
        if joint not in msg.name:
            self.publish(
                self.stream.invalid(stamp_s, "drive_joint_missing"), msg.header.stamp
            )
            return
        index = msg.name.index(joint)
        position = msg.position[index] if index < len(msg.position) else None
        velocity = msg.velocity[index] if index < len(msg.velocity) else None
        signal = msg.effort[index] if index < len(msg.effort) else None
        plugin = self.plugins.get(joint, "")
        source = (
            "mock_joint_effort"
            if "mock_components" in plugin
            else "hardware_joint_effort"
            if "dynamixel" in plugin.lower()
            else "unverified_joint_effort"
        )
        gravity = None
        if (
            self.gravity is not None
            and 0 <= now - self.gravity[0] <= self.stream.timeout_s
        ):
            gravity = self.gravity[1]
        kwargs = (
            {"effort_nm": signal}
            if self.input_unit == "torque_nm"
            else {
                "current_a": None
                if signal is None
                else signal * (0.001 if self.input_unit == "current_ma" else 1.0)
            }
        )
        if self.input_unit != "torque_nm":
            source = source.replace("effort", "current")
        state = self.stream.update(
            stamp_s=stamp_s,
            now_s=now,
            position_rad=position,
            velocity_rad_s=velocity,
            gravity_base_m_s2=gravity,
            source=source,
            **kwargs,
        )
        self.publish(state, msg.header.stamp)

    def tick(self, _event=None):
        stamp = self.now()
        invalid = self.stream.expired(seconds(stamp))
        if invalid is not None:
            self.publish(invalid, stamp)

    def publish(self, state, stamp):
        out = JointState()
        out.header.stamp = stamp
        out.header.frame_id = self.stream.frame
        out.name = [f"{self.side}_{self.stream.tool or 'unknown'}_opening"]
        # This is a virtual prismatic opening coordinate: metres and newtons.
        out.position = [state["gap_m"] if state["gap_m"] is not None else math.nan]
        out.velocity = (
            [state["gap_velocity_m_s"]] if "gap_velocity_m_s" in state else []
        )
        out.effort = [state["force_n"] if state["valid"] else math.nan]
        self.force_pub.publish(out)
        status = DiagnosticStatus()
        status.name = self.side + "/tool_force"
        status.hardware_id = self.stream.tool
        status.level = (
            DiagnosticStatus.ERROR
            if not state["valid"]
            else (
                DiagnosticStatus.OK
                if state["calibrated"]
                and state["compensation_complete"]
                and state["source"].startswith("hardware_")
                else DiagnosticStatus.WARN
            )
        )
        status.message = (
            state["reason"]
            if not state["valid"]
            else (
                "calibrated estimate"
                if status.level == DiagnosticStatus.OK
                else "uncalibrated or partially compensated estimate"
            )
        )
        status.values = [
            KeyValue(key=key, value=json.dumps(value, ensure_ascii=False))
            for key, value in state.items()
        ]
        status.values.append(KeyValue(key="input_unit", value=self.input_unit))
        message = DiagnosticArray()
        message.header = out.header
        message.status = [status]
        self.status_pub.publish(message)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("lhand", "rhand"), required=True)
    parser.add_argument("--profile-dir", type=Path)
    parser.add_argument("--timeout-s", type=float, default=0.25)
    parser.add_argument("--input-topic", default="joint_states")
    parser.add_argument("--output-topic", default="tool_force")
    parser.add_argument(
        "--input-unit",
        choices=("torque_nm", "current_a", "current_ma"),
        default="torque_nm",
    )
    args = parser.parse_args(node_argv(argv))
    node = ros_compat.init_node("tool_force_publisher")
    publisher = ToolForcePublisher(node, **vars(args))
    ros_compat.spin(node)
    return publisher


if __name__ == "__main__":
    main()

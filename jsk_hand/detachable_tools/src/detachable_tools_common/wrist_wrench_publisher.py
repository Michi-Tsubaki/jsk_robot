"""Six-axis sensor observation bridge; no dependence on tool, task or commands."""

import argparse
import json
import math
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import WrenchStamped

from .description import package_share_path
from .ros_compat import node_argv, ros_compat
from .tool_force_publisher import seconds
from .wrench_telemetry import WrenchStream


class WristWrenchPublisher:
    def __init__(
        self,
        node,
        *,
        side,
        config=None,
        input_topic=None,
        output_topic="wrist_wrench",
        timeout_s=0.25,
        log_jsonl=None,
    ):
        self.node, self.side = node, side
        profile = json.loads(
            Path(
                config or package_share_path() / "config/wrist_sensors.json"
            ).read_text()
        )
        key = "left" if side == "lhand" else "right"
        cfg = profile["sensors"][key]
        self.stream = WrenchStream(key, cfg, timeout_s)
        topic = input_topic or cfg["input_topic"]
        if topic == output_topic:
            raise ValueError("Input and output wrench topics must differ")
        self.publisher = ros_compat.create_publisher(
            node, output_topic, WrenchStamped, queue_size=10
        )
        self.status_pub = ros_compat.create_publisher(
            node, output_topic + "/status", DiagnosticArray, queue_size=10
        )
        self.subscription = ros_compat.create_subscription(
            node, topic, WrenchStamped, self.sample, queue_size=10
        )
        self.timer = ros_compat.create_timer(node, min(0.1, timeout_s / 2), self.tick)
        self.log = None
        if log_jsonl:
            path = Path(log_jsonl)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.log = path.open("a", buffering=1)

    def sample(self, message):
        w = message.wrench
        state = self.stream.update(
            stamp_s=seconds(message.header.stamp),
            now_s=seconds(ros_compat.stamp_now(self.node)),
            frame_id=message.header.frame_id,
            force_n=[w.force.x, w.force.y, w.force.z],
            torque_nm=[w.torque.x, w.torque.y, w.torque.z],
        )
        self.publish(state, message.header.stamp)

    def tick(self, _event=None):
        stamp = ros_compat.stamp_now(self.node)
        state = self.stream.expired(seconds(stamp))
        if state is not None:
            self.publish(state, stamp)

    def publish(self, state, stamp):
        out = WrenchStamped()
        out.header.stamp, out.header.frame_id = stamp, state["frame_id"]
        f = state["force_n"] if state["valid"] else [math.nan] * 3
        t = state["torque_nm"] if state["valid"] else [math.nan] * 3
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = f
        out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = t
        self.publisher.publish(out)
        status = DiagnosticStatus()
        status.name = self.side + "/wrist_wrench"
        status.hardware_id = state["frame_id"]
        status.level = (
            DiagnosticStatus.ERROR
            if not state["valid"]
            else (DiagnosticStatus.OK if state["calibrated"] else DiagnosticStatus.WARN)
        )
        status.message = (
            state["reason"]
            if not state["valid"]
            else (
                "calibrated sensor wrench"
                if state["calibrated"]
                else "sensor calibration unverified"
            )
        )
        status.values = [
            KeyValue(key=k, value=json.dumps(v, allow_nan=False))
            for k, v in state.items()
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header, diagnostics.status = out.header, [status]
        self.status_pub.publish(diagnostics)
        if self.log:
            self.log.write(json.dumps(state, allow_nan=False) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("lhand", "rhand"), required=True)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--input-topic")
    parser.add_argument("--output-topic", default="wrist_wrench")
    parser.add_argument("--timeout-s", type=float, default=0.25)
    parser.add_argument("--log-jsonl", type=Path)
    args = parser.parse_args(node_argv(argv))
    node = ros_compat.init_node("wrist_wrench_publisher")
    publisher = WristWrenchPublisher(node, **vars(args))
    try:
        ros_compat.spin(node)
    finally:
        if publisher.log:
            publisher.log.close()


if __name__ == "__main__":
    main()

"""Exercise both sensor transports in an isolated domain, without motor commands."""

import json
import math
import os
import time
import unittest
from pathlib import Path


@unittest.skipUnless(os.environ.get("ROS_VERSION") == "2", "ROS 2 required")
class WristTransportTest(unittest.TestCase):
    def test_both_sides_frames_and_timeout(self):
        import rclpy
        from detachable_tools_common.wrist_wrench_publisher import WristWrenchPublisher
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import WrenchStamped
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init(args=[])
        executor = SingleThreadedExecutor()
        root = "/wrist_contract_" + str(os.getpid())
        fixture = rclpy.create_node("fixture", namespace=root)
        executor.add_node(fixture)
        nodes, adapters, pubs, subs = [], [], {}, []
        received, statuses = {}, {}
        config_path = Path(__file__).resolve().parents[1] / "config/wrist_sensors.json"
        config = json.loads(config_path.read_text())

        def until(predicate):
            deadline = time.monotonic() + 3
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            self.assertTrue(predicate(), "ROS transport timed out")

        try:
            for side in ("lhand", "rhand"):
                node = rclpy.create_node("wrist", namespace=root + "/" + side)
                nodes.append(node)
                executor.add_node(node)
                adapters.append(
                    WristWrenchPublisher(
                        node, side=side, config=config_path, timeout_s=0.3
                    )
                )
                pubs[side] = fixture.create_publisher(
                    WrenchStamped, side + "/wrench_raw", 10
                )
                subs.append(
                    fixture.create_subscription(
                        WrenchStamped,
                        side + "/wrist_wrench",
                        lambda msg, side=side: received.__setitem__(side, msg),
                        10,
                    )
                )
                subs.append(
                    fixture.create_subscription(
                        DiagnosticArray,
                        side + "/wrist_wrench/status",
                        lambda msg, side=side: statuses.__setitem__(
                            side,
                            {v.key: json.loads(v.value) for v in msg.status[0].values},
                        ),
                        10,
                    )
                )
            until(
                lambda: (
                    all(p.get_subscription_count() for p in pubs.values())
                    and all(a.publisher.get_subscription_count() for a in adapters)
                )
            )
            for side, key in [("lhand", "left"), ("rhand", "right")]:
                msg = WrenchStamped()
                msg.header.stamp = fixture.get_clock().now().to_msg()
                msg.header.frame_id = config["sensors"][key]["input_frame"]
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (
                    1.0,
                    2.0,
                    3.0,
                )
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (
                    0.1,
                    0.2,
                    0.3,
                )
                pubs[side].publish(msg)
            until(lambda: all(statuses.get(s, {}).get("valid") for s in pubs))
            until(
                lambda: all(
                    s in received and math.isfinite(received[s].wrench.force.x)
                    for s in pubs
                )
            )
            for side in pubs:
                self.assertEqual(statuses[side]["force_n"], [1.0, 2.0, 3.0])
                self.assertEqual(statuses[side]["torque_nm"], [0.1, 0.2, 0.3])
                self.assertEqual(statuses[side]["source"], "external_wrench_unverified")
                self.assertFalse(statuses[side]["calibrated"])
                self.assertEqual(received[side].wrench.torque.z, 0.3)
            msg.header.frame_id = "wrong_sensor_frame"
            msg.header.stamp = fixture.get_clock().now().to_msg()
            pubs["rhand"].publish(msg)
            until(lambda: statuses["rhand"]["reason"] == "unexpected_frame")
            until(lambda: math.isnan(received["rhand"].wrench.force.x))
            until(lambda: all(statuses[s]["reason"] == "sample_timeout" for s in pubs))
            until(lambda: all(math.isnan(received[s].wrench.torque.z) for s in pubs))
        finally:
            executor.shutdown()
            for node in nodes + [fixture]:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

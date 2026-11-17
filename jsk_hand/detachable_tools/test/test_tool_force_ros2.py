"""ROS 2 transport contract. Run on an isolated ROS_DOMAIN_ID; no motor commands."""

import json
import math
import os
import time
import unittest
from pathlib import Path


@unittest.skipUnless(os.environ.get("ROS_VERSION") == "2", "ROS 2 environment required")
class ToolForceTransportTest(unittest.TestCase):
    def test_effort_provenance_missing_signal_detach_and_timeout(self):
        import rclpy
        from detachable_tools_common.force_estimation import interpolate_profile
        from detachable_tools_common.tool_force_publisher import ToolForcePublisher
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from geometry_msgs.msg import Vector3Stamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        rclpy.init(args=[])
        namespace = "/tool_force_test_" + str(os.getpid())
        target = rclpy.create_node("estimator", namespace=namespace)
        io = rclpy.create_node("fixture", namespace=namespace)
        executor = SingleThreadedExecutor()
        executor.add_node(target)
        executor.add_node(io)
        directory = Path(__file__).resolve().parents[1] / "config/tool_force"
        adapter = ToolForcePublisher(
            target, side="rhand", profile_dir=directory, timeout_s=0.3
        )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        selection = io.create_publisher(DiagnosticArray, "tool_state", qos)
        description = io.create_publisher(String, "robot_description", qos)
        joints = io.create_publisher(JointState, "joint_states", 10)
        gravity = io.create_publisher(Vector3Stamped, "tool_gravity", 10)
        forces, states = [], []
        subscriptions = [
            io.create_subscription(JointState, "tool_force", forces.append, 10),
            io.create_subscription(
                DiagnosticArray, "tool_force/status", states.append, 10
            ),
        ]

        def until(predicate, timeout=3):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            self.assertTrue(predicate(), "ROS transport timed out")

        def state():
            return (
                {
                    v.key: json.loads(v.value)
                    for v in states[-1].status[0].values
                    if v.key != "input_unit"
                }
                if states
                else {}
            )

        def attach(attached):
            values = {
                "side": "rhand",
                "tool": "pean",
                "attached": str(attached).lower(),
                "command_joint": "test_motor",
                "root_link": "test_tool_mount",
            }
            selection.publish(
                DiagnosticArray(
                    status=[
                        DiagnosticStatus(
                            values=[KeyValue(key=k, value=v) for k, v in values.items()]
                        )
                    ]
                )
            )

        try:
            until(
                lambda: (
                    joints.get_subscription_count() == 1
                    and selection.get_subscription_count() == 1
                    and adapter.force_pub.get_subscription_count() == 1
                )
            )
            attach(True)
            description.publish(
                String(
                    data='<robot name="test"><ros2_control name="test" type="system">'
                    "<hardware><plugin>mock_components/GenericSystem</plugin></hardware>"
                    '<joint name="test_motor"/></ros2_control></robot>'
                )
            )
            until(lambda: adapter.stream.attached and "test_motor" in adapter.plugins)
            zero_g = Vector3Stamped()
            zero_g.header.stamp = io.get_clock().now().to_msg()
            zero_g.header.frame_id = "test_tool_mount"
            gravity.publish(zero_g)
            until(lambda: adapter.gravity is not None)
            profile = json.loads((directory / "pean.json").read_text())
            jac = interpolate_profile(profile, "right", -0.1)["jacobian_m_per_rad"]
            sample = JointState(
                name=["test_motor"], position=[-0.1], velocity=[0.0], effort=[2 * jac]
            )
            sample.header.stamp = io.get_clock().now().to_msg()
            joints.publish(sample)
            until(lambda: state().get("valid") is True)
            until(lambda: bool(forces) and math.isfinite(forces[-1].effort[0]))
            self.assertAlmostEqual(forces[-1].effort[0], 2.0, places=6)
            self.assertEqual(state()["source"], "mock_joint_effort")
            self.assertFalse(state()["calibrated"])
            self.assertEqual(forces[-1].header.frame_id, "test_tool_mount")
            sample.effort = []
            sample.header.stamp = io.get_clock().now().to_msg()
            joints.publish(sample)
            until(lambda: state().get("reason") == "effort_unavailable")
            until(lambda: math.isnan(forces[-1].effort[0]))
            attach(False)
            until(lambda: state().get("reason") == "tool_detached")
            attach(True)
            until(lambda: adapter.stream.attached)
            sample.effort = [2 * jac]
            sample.header.stamp = io.get_clock().now().to_msg()
            joints.publish(sample)
            until(lambda: state().get("valid") is True)
            until(lambda: state().get("reason") == "sample_timeout")
            until(lambda: math.isnan(forces[-1].effort[0]))
            self.assertIsNone(state()["force_n"])
        finally:
            del subscriptions
            executor.shutdown()
            target.destroy_node()
            io.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

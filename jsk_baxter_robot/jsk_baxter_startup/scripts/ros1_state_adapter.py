#!/usr/bin/env python
"""Run beside the Baxter SDK. Export custom state using bridgeable ROS messages."""
import copy
import math
import rospy
from baxter_core_msgs.msg import AssemblyState, EndEffectorState, EndpointState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def main():
    rospy.init_node('baxter_standard_state')
    publishers, subscribers = [], []
    enabled = rospy.Publisher('/baxter/robot_enabled', Bool, queue_size=1)
    subscribers.append(rospy.Subscriber('/robot/state', AssemblyState,
                                        lambda m: enabled.publish(Bool(data=m.enabled)), queue_size=1))
    for side in ('left', 'right'):
        pose_pub = rospy.Publisher('/baxter/' + side + '/endpoint_pose', PoseStamped, queue_size=1)
        grip_pub = rospy.Publisher('/baxter/' + side + '/gripper_state', JointState, queue_size=1)
        publishers.extend([pose_pub, grip_pub])

        def pose_cb(msg, pub=pose_pub):
            out = PoseStamped(header=copy.deepcopy(msg.header), pose=msg.pose)
            if not out.header.frame_id:
                out.header.frame_id = 'base'
            pub.publish(out)

        def grip_cb(msg, pub=grip_pub, arm=side):
            # Electric gripper only: SDK reports 0 (closed) ... 100 (open).
            if msg.error or msg.calibrated != EndEffectorState.STATE_TRUE or (math.isnan(msg.position) or math.isinf(msg.position)):
                return
            if not 0 <= msg.position <= 100:
                return
            out = JointState()
            out.header.stamp = msg.timestamp
            if out.header.stamp == rospy.Time():
                out.header.stamp = rospy.Time.now()
            out.name = [arm + '_gripper_opening']
            out.position = [msg.position / 100.0]
            pub.publish(out)

        subscribers.extend([
            rospy.Subscriber('/robot/limb/' + side + '/endpoint_state', EndpointState, pose_cb, queue_size=1),
            rospy.Subscriber('/robot/end_effector/' + side + '_gripper/state', EndEffectorState, grip_cb, queue_size=1),
        ])
    rospy.spin()


if __name__ == '__main__':
    main()

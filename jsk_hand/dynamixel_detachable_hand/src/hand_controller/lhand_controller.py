#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class LHandInterface:
    DETACH_TORQUE_CONSTANT = 1.15
    DETACH_CURRENT_A = 0.4
    HAND_EFFORT_LIMIT = 0.92

    def __init__(self, groupname="lhand"):
        self.groupname = groupname
        self.action_client = actionlib.SimpleActionClient(
            "/lhand/position_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self.detach_effort_pub = rospy.Publisher(
            "/lhand/joint_group_effort_controller/command",
            Float64MultiArray,
            queue_size=1
        )
        if not self.action_client.wait_for_server(rospy.Duration(5)):
            rospy.logwarn("Action server not available")
        self.joint_states = {}
        rospy.Subscriber("/lhand/joint_states", JointState, self._joint_states_callback)
        
    def _joint_states_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos
    
    def get_joint_state(self, joint_name):
        return self.joint_states.get(joint_name)
    
    def move_hand(self, grasp_angle, wait=True, tm=1.0, velocity=0.0, acceleration=0.0, effort=0.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["lhand_joint"]
        point = JointTrajectoryPoint()
        point.positions = [grasp_angle]
        point.velocities = [velocity]
        point.accelerations = [acceleration]
        point.effort = [effort]
        point.time_from_start = rospy.Duration(tm)
        goal.trajectory.points = [point]
        self.action_client.send_goal(goal)
        if wait:
            self.action_client.wait_for_result(rospy.Duration(tm + 5.0))
            state = self.action_client.get_state()
            if state != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn("Hand movement failed: {}".format(state))
            return state

    def move_detach_joint(self, angle, wait=True, tm=1.0, velocity=0.3, acceleration=0.0, effort=0.0):
        current_ma = effort / self.DETACH_TORQUE_CONSTANT * 1000.0
        self.command_detach_current(current_ma, wait=wait, tm=tm)
        if wait:
            return actionlib.GoalStatus.SUCCEEDED

    def _detach_effort_command(self, detach_effort):
        joints = rospy.get_param(
            "/{}/joint_group_effort_controller/joints".format(self.groupname),
            ["lhand_detach_joint_0"]
        )
        data = []
        for joint in joints:
            if joint == "lhand_detach_joint_0":
                data.append(detach_effort)
            else:
                data.append(self.HAND_EFFORT_LIMIT)
        return data

    def set_detach_effort(self, detach_effort):
        msg = Float64MultiArray()
        msg.data = self._detach_effort_command(detach_effort)
        self.detach_effort_pub.publish(msg)

    def stop_detach(self):
        self.set_detach_effort(0.0)

    def command_detach_current(self, current_ma, wait=True, tm=1.0):
        detach_effort = (current_ma / 1000.0) * self.DETACH_TORQUE_CONSTANT
        self.set_detach_effort(detach_effort)
        if tm is not None and tm > 0.0:
            if wait:
                rospy.sleep(tm)
                self.stop_detach()
            else:
                rospy.Timer(rospy.Duration(tm), lambda event: self.stop_detach(), oneshot=True)
    
    def cancel_move_hand(self):
        self.action_client.cancel_goal()
    
    def hand_moving_p(self):
        return self.action_client.get_state() == actionlib.GoalStatus.ACTIVE
    
    def open(self, wait=True, tm=1.0, velocity=2.0, acceleration=0.0, effort=0.0):
        return self.move_hand(0.0, wait, tm, velocity, acceleration, effort)
    
    def close(self, wait=True, tm=1.0, velocity=2.0, acceleration=0.0, effort=0.0):
        return self.move_hand(-2.7, wait, tm, velocity, acceleration, effort)

    def attach(self, wait=True, tm=1.0, velocity=0.3, acceleration=0.0, effort=None):
        return self.command_detach_current(-400.0, wait=wait, tm=tm)

    def detach(self, wait=True, tm=1.0, velocity=0.3, acceleration=0.0, effort=None):
        return self.command_detach_current(400.0, wait=wait, tm=tm)
    
    def wait_for_hand(self):
        self.action_client.wait_for_result()

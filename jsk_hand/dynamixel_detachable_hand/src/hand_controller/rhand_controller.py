#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class RHandInterface:
    def __init__(self, groupname="rhand"):
        self.groupname = groupname
        self.action_client = actionlib.SimpleActionClient(
            "/rhand/position_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        if not self.action_client.wait_for_server(rospy.Duration(5)):
            rospy.logwarn("Action server not available")
        self.joint_states = {}
        rospy.Subscriber("/rhand/joint_states", JointState, self._joint_states_callback)
        
    def _joint_states_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos
    
    def get_joint_state(self, joint_name):
        return self.joint_states.get(joint_name)
    
    def move_hand(self, grasp_angle, wait=True, tm=1.0, velocity=0.0, acceleration=0.0, effort=0.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["rhand_joint"]
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
    
    def cancel_move_hand(self):
        self.action_client.cancel_goal()
    
    def hand_moving_p(self):
        return self.action_client.get_state() == actionlib.GoalStatus.ACTIVE
    
    def open(self, wait=True, tm=1.0, velocity=2.0, acceleration=0.0, effort=0.0):
        return self.move_hand(0.0, wait, tm, velocity, acceleration, effort)
    
    def close(self, wait=True, tm=1.0, velocity=2.0, acceleration=0.0, effort=0.0):
        return self.move_hand(-2.7, wait, tm, velocity, acceleration, effort)
    
    def open_holder(self, wait=True, tm=1.0, velocity=0.5, acceleration=0.0, effort=0.0):
        return self.move_hand(-0.1, wait, tm, velocity, acceleration, effort)
    
    def close_holder(self, wait=True, tm=1.0, velocity=0.5, acceleration=0.0, effort=0.0):
        return self.move_hand(0.08, wait, tm, velocity, acceleration, effort)
    
    def wait_for_hand(self):
        self.action_client.wait_for_result()

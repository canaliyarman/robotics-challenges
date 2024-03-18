#!/usr/bin/env python

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus


class GoalSender:

    def __init__(self):

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.currently_moving = False
        # Wait for Action Client to come online
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server() # rospy.Duration(5.0)
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        # Action Server Connected
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

    def active_cb(self):
        rospy.loginfo("Goal pose  is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        rospy.loginfo("Feedback for goal pose received")
        # if feedback.status != 1:
        #     rospy.loginfo("CANCELLING GOAL")
        #     rospy.loginfo(str(feedback))


    def done_cb(self, status, result):
        # Reference for terminal status values: http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html

        if status == 0:
            rospy.loginfo("The goal has yet to be processed by the action server")

        if status == 1:
            rospy.loginfo("The goal is currently being processed by the action server!")

        if status == 2:
            rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")
            self.currently_moving = False
        if status == 3:
            rospy.loginfo("Goal pose reached")
            self.currently_moving = False
        if status == 4:
            rospy.loginfo("Goal pose was aborted by the Action Server")
            self.currently_moving = False
            self.client.cancel_goal()
        if status == 5:
            rospy.loginfo("Goal pose has been rejected by the Action Server")
            self.currently_moving = False
            self.client.cancel_goal()
        if status == 6:
            rospy.loginfo("The goal received a cancel request after it started executing and has not yet completed execution")
            self.currently_moving = False
            self.client.cancel_goal()
        if status == 7:
            rospy.loginfo("The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled")
            self.currently_moving = False
            self.client.cancel_goal()
        if status == 8:
            rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")
            self.currently_moving = False
            self.client.cancel_goal()
        if status == 9:
            rospy.loginfo("An action client can determine that a goal is LOST. This should not be sent over the wire by an action server")
            self.currently_moving = False
            self.client.cancel_goal()
    def send_goal(self, goal=PoseStamped()):

        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header = goal.header
        goal_msg.target_pose.pose = goal.pose

        #Set Msg
        rospy.loginfo("Sending goal pose "+str(goal_msg)+" to Action Server")
        self.currently_moving = True
        self.client.send_goal(goal_msg, self.done_cb, self.active_cb, self.feedback_cb)
        
    def cancel_goal(self):
        rospy.loginfo("EXPLORER CANCELLED THE ACTION, BAD ROUTE")
        self.client.cancel_goal()
        self.currently_moving = False
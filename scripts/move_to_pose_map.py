#!/usr/bin/env python

import sys
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry import rotate_pose_msg_by_euler_angles as rotate 
from math import pi
# from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist

class GoToPose():
    def __init__(self):
        # rospy.init_node('nav_test', anonymous=False)

        #what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self._shutdown)
        
        #tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("waiting for the action server to come up...")
        #allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        #we'll send a goal to the robot to tell it to move to a pose that's near the docking station
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'odom'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        
    def move_to_pose(self, x1, y1, th):
        # Goal
        self.goal.target_pose.pose.position.x = x1
        self.goal.target_pose.pose.position.y = y1
        self.goal.target_pose.pose.position.z = 0.000
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.000
        self.goal.target_pose.pose.orientation.z = 0.000
        self.goal.target_pose.pose.orientation.w = 0.000

        # th = th*(pi/180.0)
        
        #start moving
        self.move_base.send_goal(self.goal)
        rospy.loginfo("moving to desired position...")
        #allow TurtleBot up to 60 seconds to complete task
        self.success = self.move_base.wait_for_result(rospy.Duration(60)) 

        if not self.success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose :(")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, reached! :)")

    def _shutdown(self):
        rospy.loginfo("Stopped")


if __name__ == '__main__':
    try:
        GoToPose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
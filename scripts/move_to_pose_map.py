#!/usr/bin/env python

import sys
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

class GoToPose():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

        #what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        
        #tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("waiting for the action server to come up...")
        #allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        #we'll send a goal to the robot to tell it to move to a pose that's near the docking station
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        # self.move_to_pose(-12.1, -4.5, 0.9, 0.4)

        # # # Goal 1
        # # goal.target_pose.pose = Pose(Point(-12.1, -4.5, 0.000), Quaternion(0.000, 0.000, 0.9, 0.4))
        # # #start moving
        # # self.move_base.send_goal(goal)
        # # #allow TurtleBot up to 120 seconds to complete task
        # # success = self.move_base.wait_for_result(rospy.Duration(120)) 

        # # Goal 2
        # goal.target_pose.pose = Pose(Point(-12.3, 0.26, 0.000), Quaternion(0.000, 0.000, 0.6, 0.8))        
        # #start moving
        # self.move_base.send_goal(goal)
        # #allow TurtleBot up to 120 seconds to complete task
        # success = self.move_base.wait_for_result(rospy.Duration(120)) 

        # # Goal 3
        # goal.target_pose.pose = Pose(Point(1.888, -3.2, 0.000), Quaternion(0.000, 0.000, -0.74, 0.68))        
        # #start moving
        # self.move_base.send_goal(goal)
        # #allow TurtleBot up to 120 seconds to complete task
        # success = self.move_base.wait_for_result(rospy.Duration(120)) 


        # if not self.success:
        #     self.move_base.cancel_goal()
        #     rospy.loginfo("The base failed to reach the desired pose :(")
        # else:
        #     # We made it!
        #     state = self.move_base.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         rospy.loginfo("Hooray, reached the desired pose :)")

    def move_to_pose(self, p1, p2, q3, q4):
        # Goal 1
        self.goal.target_pose.pose = Pose(Point(p1, p2, 0.000), Quaternion(0.000, 0.000, q3, q4))
        #start moving
        self.move_base.send_goal(self.goal)
        rospy.loginfo("moving to desired position...")
        #allow TurtleBot up to 120 seconds to complete task
        self.success = self.move_base.wait_for_result(rospy.Duration(120)) 

        if not self.success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose :(")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, reached! :)")

    def shutdown(self):
        rospy.loginfo("Stopped")


if __name__ == '__main__':
    try:
        GoToPose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
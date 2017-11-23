#!/usr/bin/env python


import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        quaternions = list()
        q_angle = quaternion_from_euler(0, 0, 3.14, axes='sxyz')
        q = Quaternion(*q_angle)
        quaternions.append(q)
        waypoints = list()
        waypoints.append(Pose(Point(5.0, 0.0, 0.0), quaternions[0]))
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(1))    
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoints[0]
        #goal.target_pose.pose.position = [5.0, 0.0, 0.0]
        #goal.target_pose.pose.orientation = [0.0, 0.0, 0.0,1.0]
        self.move_base.send_goal(goal)
        #self.move(goal)
    #def callback(data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #def move(self, goal):
            #self.move_base.send_goal(goal)

            #state = self.move_base.get_state()
           # if state == GoalStatus.SUCCEEDED:
               # rospy.loginfo("Goal succeeded!")
if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

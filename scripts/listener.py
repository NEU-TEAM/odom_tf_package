#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # px=data.position.x
    # py=data.position.y
    # pz=data.position.z
    # qx=data.orientation.x
    # qy=data.orientation.y
    # qz=data.orientation.z
    # qw=data.orientation.w

    px = data.position
    qx = data.orientation
    quaternions = list()
    q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
    q = Quaternion(*q_angle)
    quaternions.append(q)
    waypoints = list()
    waypoints.append(Pose(px, q))
    # waypoints.append(Pose(Point(5.0, 0.0, 0.0), quaternions[0]))
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(1))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = waypoints[0]
    # goal.target_pose.pose.position = [5.0, 0.0, 0.0]
    # goal.target_pose.pose.orientation = [0.0, 0.0, 0.0,1.0]
    move_base.send_goal(goal)
    # i=i+1


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("nav_location_goal", Pose, callback)
    # MoveBaseSquare()
    # i=0
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

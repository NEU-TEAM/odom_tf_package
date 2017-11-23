#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
import time
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

pub_task = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pub = rospy.Publisher('task_status', String, queue_size=1)

# rospy.Rate(10)
twist = Twist()
twist1 = Twist()


def callback(data):
    twist.linear.x = data.pose.position.x
    twist.linear.y = data.pose.position.y
    if twist.linear.x > 0.02:
        twist1.linear.x = 0.08
        twist1.linear.y = 0.00
    elif twist.linear.x < -0.02:
        twist1.linear.x = -0.08
        twist1.linear.y = 0.00
    else:
        twist1.linear.x = 0.0
        if twist.linear.y > 0.01:
            twist1.linear.y = 0.08

        elif twist.linear.y < 0.01:
            twist1.linear.y = -0.08

        else:
            twist1.linear.y = 0.0
            twist1.linear.x = 0.0

    pub_task.publish(twist1)


def listener():
    rospy.init_node('offset_listener', anonymous=True)
    rospy.Subscriber("/vision/grasp/location", PoseStamped, callback, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

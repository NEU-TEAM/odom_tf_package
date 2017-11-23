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


def callback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    tx = abs(x / 0.08)+2
    ty = abs(y / 0.08)+2
    twist = Twist()
    # twist.linear.z=tx
    # twist.linear.y=ty
    if x > 0:
        twist.linear.x = 0.08
    else:
        twist.linear.x = -0.08
    pub_task.publish(twist)
    rospy.sleep(100)
    twist.linear.x = 0.0
    pub_task.publish(twist)
    if y > 0:
        twist.angular.z = 0.08
    else:
        twist.angular.z = -0.08
    pub_task.publish(twist)
    rospy.sleep(157)
    twist.angular.z = 0.0
    pub_task.publish(twist)

    twist.linear.x = 0.0
    pub_task.publish(twist)
    rospy.sleep(100)
    twist.linear.x = 0.0
    pub_task.publish(twist)
    if y < 0:
        twist.angular.z = 0.08
    else:
        twist.angular.z = -0.08

    pub_task.publish(twist)
    rospy.sleep(157)
    twist.angular.z = 0.0

    pub_task.publish(twist)
    pub.publish("Goal succeeded!")


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/vision/grasp/location", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

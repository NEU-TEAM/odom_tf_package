#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped, Twist

pub_task = rospy.Publisher('cmd_vel', Twist, queue_size=1)

base_vel = Twist()


def callback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    if x > 0.02:
        base_vel.linear.x = 0.08
        base_vel.linear.y = 0.00
    elif x < -0.02:
        base_vel.linear.x = -0.08
        base_vel.linear.y = 0.00
    else:
        base_vel.linear.x = 0.0
        if y > 0.01:
            base_vel.linear.y = 0.08
        elif y < 0.01:
            base_vel.linear.y = -0.08
        else:
            base_vel.linear.y = 0.0
            base_vel.linear.x = 0.0

    pub_task.publish(base_vel)


def listener():
    rospy.init_node('offset_listener', anonymous=True)
    rospy.Subscriber("/ctrl/vision/grasp/location", PoseStamped, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    listener()

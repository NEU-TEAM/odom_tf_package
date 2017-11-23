#!/usr/bin/env python
# license removed for brevity
import rospy
import time

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist


def talker():
    pub = rospy.Publisher('nav_location_goal', Pose, queue_size=10)
    rospy.init_node('pub_location', anonymous=True)
    rate = rospy.Rate(0.005) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)

        pose = Pose(Point(5.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1))
        pub.publish(pose)
        # rate.sleep()
        # rospy.sleep(10)
        # time.sleep(5)
        # pose=Pose(Point(0.5, 3, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        # pub.publish(pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

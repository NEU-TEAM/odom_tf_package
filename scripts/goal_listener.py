#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

pub_status = rospy.Publisher('/feed/base/task_status', Int8, queue_size=1)


def callback(data):
    # Fill in position and quaternion into way points
    px = data.pose.position

    q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
    q = Quaternion(*q_angle)

    way_points = list()
    way_points.append(Pose(px, q))

    # DEBUG
    # way_points.append(Pose(Point(5.0, 0.0, 0.0), q)

    # Initialize move base action
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(1))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = way_points[0]

    # Move to goal and get result
    move_base.send_goal(goal)
    finished_within_time = move_base.wait_for_result(rospy.Duration(1200))

    status = Int8()  # Store feedback
    status.data = 0  # Initialize with 0
    if not finished_within_time:
        move_base.cancel_goal()
        status.data = -1
        rospy.loginfo("Base: Time out for achieving goal")
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            status.data = 1
            rospy.loginfo("Base: Goal succeeded!")
        elif state == GoalStatus.ABORTED:
            status.data = -1
    pub_status.publish(status)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/ctrl/voice/nav_location_goal", PoseStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

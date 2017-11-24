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
    px = data.pose.position

    quaternions = list()
    q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
    q = Quaternion(*q_angle)
    quaternions.append(q)
    way_points = list()
    way_points.append(Pose(px, q))

    # way_points.append(Pose(Point(5.0, 0.0, 0.0), quaternions[0]))
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(1))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = way_points[0]
    # goal.target_pose.pose.position = [5.0, 0.0, 0.0]
    # goal.target_pose.pose.orientation = [0.0, 0.0, 0.0,1.0]
    move_base.send_goal(goal)
    finished_within_time = move_base.wait_for_result(rospy.Duration(1200))
    # move(goal)
    status = Int8()  # For feedback
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

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32, Int8, Float32
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from tf.transformations import quaternion_from_euler
from math import pow, sqrt
import time

move_cmd = Twist()

q_angle1 = quaternion_from_euler(0, 0, 1.57, axes='sxyz')
q1 = Quaternion(*q_angle1)
q_angle2 = quaternion_from_euler(0, 0, 0, axes='sxyz')
q2 = Quaternion(*q_angle2)
q_angle3 = quaternion_from_euler(0, 0, -1.57, axes='sxyz')
q3 = Quaternion(*q_angle3)
locations = dict()
locations['Table1'] = Pose(Point(2.66, -2.4, 0), Quaternion(0.000, 0.000, -0.707, 0.707))
locations['Table2'] = Pose(Point(2.66, -4.0, 0), Quaternion(0.000, 0.000, -0.707, 0.707))
locations['Table3'] = Pose(Point(2.81, -0.8, 0.000), q2)
locations['Table4'] = Pose(Point(0.86, 0.000, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
locations['Sofa1'] = Pose(Point(0.86, 0.000, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
locations['Sofa2'] = Pose(Point(1.86, -0.8, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
locations['Sofa3'] = Pose(Point(2.26, 0.000, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
locations['Sofa4'] = Pose(Point(2.66, -2.4, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
locations['charge'] = Pose(Point(2.26, 0.000, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))

armlocations = dict()
armlocations['Larmup'] = Pose(Point(1, 0, 0), Quaternion(0.000, 0.000, 0, 1))
armlocations['Larmdown'] = Pose(Point(1, 0, 0), Quaternion(0.000, 0.000, 0, 1))


def callback1(data):
    global yaw
    yaw = data.data


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    cmd = data.data
    move_cmd = Twist()
    if cmd == "Forward":
        move_cmd.linear.x = 0.2
        pubspeed.publish(move_cmd)
    elif cmd == "Backward":
        move_cmd.linear.x = -0.2
        pubspeed.publish(move_cmd)
    elif cmd == "Left":
        # move_cmd = Twist()
        # move_cmd.linear.x = -0.2
        move_cmd.angular.z = 0.5
        pubspeed.publish(move_cmd)
    elif cmd == "Right":
        # move_cmd=Twist()
        move_cmd.angular.z = -0.5
        pubspeed.publish(move_cmd)
    elif cmd == "Stop":
        move_cmd = Twist()
        pubspeed.publish(move_cmd)
    elif cmd == "Table1":
        sendtask('Table1')
    elif cmd == "Table2":
        sendtask('Table2')
    elif cmd == "Table3":
        sendtask('Table3')
    elif cmd == "Table4":
        sendtask('Table4')
    elif cmd == "Sofa1":
        sendtask('Sofa1')
    elif cmd == "Sofa2":
        # sendtask('Sofa2')
        pubtask.publish(1.0)
    elif cmd == "Sofa3":
        sendtask('Sofa3')
    elif cmd == "Sofa4":
        sendtask('Sofa4')
    elif cmd == "Larmup":
        pubarmup.publish(0)
    elif cmd == "Larmdown":
        pubarmdown.publish(0)


# dingyifabudehuati
pub = rospy.Publisher('chatter', String, queue_size=10)
pubarmup = rospy.Publisher('Larmup', Int32, queue_size=10)
pubarmdown = rospy.Publisher('Larmdown', Int32, queue_size=10)
pubtask = rospy.Publisher('arduino_sudu/taskname', Float32, queue_size=10)
pubspeed = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pubvision = rospy.Publisher('task/vision', Int8, queue_size=10)
pubtable = rospy.Publisher('arduino_sudu/tabletask', Int8, queue_size=10)
pubcharge = rospy.Publisher('arduino_sudu/chargetask', Int8, queue_size=10)


def sendtask(location):
    # type: (object) -> object
    # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    # Subscribe to the move_base action server
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # move_base.cancel_goal()
    rospy.loginfo("Waiting for move_base action server...")

    # Wait 60 seconds for the action server to become available
    move_base.wait_for_server(rospy.Duration(1200))

    rospy.loginfo("Connected to move base server")

    goal = MoveBaseGoal()
    goal.target_pose.pose = locations[location]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Going to: " + str(location))
    move_base.send_goal(goal)
    finished_within_time = move_base.wait_for_result(rospy.Duration(1200))
    # move(goal)
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.loginfo("Timed out achieving goal")
    else:
        # We made it!
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            if 'Table' in location:
                for a in range(1, 2):
                    time.sleep(2)
                    pubtable.publish(2)
                    # pubvision.publish(1)
            elif 'charge' in location:
                for a in range(1, 2):
                    time.sleep(2)
                    pubcharge.publish(3)
                    # pubvision.publish(1)
            elif 'Sofa' in location:
                global yaw
                for a in range(1, 2):
                    time.sleep(2)
                    pubtask.publish(1.0)
                    # pubtask.publish(1.0)
            rospy.loginfo("Goal succeeded!")


def listener():
    rospy.init_node('cmdlistener', anonymous=True)
    rospy.Subscriber("cmdtalker", String, callback)
    rospy.Subscriber("arduino_jiaodu/Yaw", Float32, callback1)
    rospy.spin()


if __name__ == '__main__':
    listener()

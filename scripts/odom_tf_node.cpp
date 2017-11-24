#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include "geometry_msgs/Twist.h"

double track_speedL1;
double track_speedR1;
double track_speedL2;
double track_speedR2;


double yaw;

void sudu1_Callback(const geometry_msgs::Twist& msg)
{
  track_speedL1=msg.linear.x;
  track_speedR1=msg.linear.y;
  track_speedL2=msg.linear.z;
  track_speedR2=msg.angular.x;
}
/*void sudu2_Callback(const std_msgs::Float32& msg)
{
    track_speedR1=msg.data;
}
void sudu3_Callback(const std_msgs::Float32& msg)
{
    track_speedL2=msg.data;
}
void sudu4_Callback(const std_msgs::Float32& msg)
{
    track_speedR2=msg.data;
}*/

void jiaodu_Callback(const std_msgs::Float32& msg)
{
  yaw=msg.data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  
  /* ros::Subscriber<std_msgs::Float32> sub("hj_track_speedL", &sudu1_Callback);
    ros::Subscriber<std_msgs::Float32> sub1("hj_track_speedR", &sudu2_Callback);*/
  
  ros::Subscriber sub = n.subscribe("stm_publish", 10000, sudu1_Callback);
  /*ros::Subscriber sub1 = n.subscribe("arduino_sudu/hj_track_speedR1", 10000, sudu2_Callback);
    ros::Subscriber sub2 = n.subscribe("arduino_sudu/hj_track_speedL2", 10000, sudu3_Callback);
    ros::Subscriber sub3 = n.subscribe("arduino_sudu/hj_track_speedR2", 10000, sudu4_Callback);*/
  
  
  ros::Subscriber sub4 = n.subscribe("arduino_jiaodu/Yaw", 10000, jiaodu_Callback);
  
  double lyaw=0.0;
  
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(1.0);
  while(n.ok())
  {
    ros::spinOnce(); // check for incoming messages
    current_time = ros::Time::now();
    
    // Compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double cyaw = yaw;
    double pjv = (track_speedL1-track_speedR1) / 2; //两侧速度计算的平台质心速度
    double pjs = pjv * dt;
    
    double delta_x =pjs*cos(cyaw);
    double delta_y =pjs*sin(cyaw);
    x += delta_x;
    y += delta_y;
    th = cyaw;
    vx = pjv;
    vth = (cyaw-lyaw) / dt;
    
    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    // Send the transform
    odom_broadcaster.sendTransform(odom_trans);
    
    // Next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    
    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;
    lyaw = cyaw;
    r.sleep();
  }
  return 0;
}

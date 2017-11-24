#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// Wheel speed
double front_left = 0.0;
double front_right = 0.0;
double back_left = 0.0;
double back_right = 0.0;

// Yaw value of base from IMU
double yaw_imu_ = 0.0;

void speedCallback(const geometry_msgs::Twist& msg)
{
  front_left = msg.linear.x;
  front_right = msg.linear.y;
  back_left = msg.linear.z;
  back_right = msg.angular.x;
}

void rotationCallback(const std_msgs::Float32& msg)
{
  yaw_imu_ = msg.data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Rate loop_rate(20);

  ros::Subscriber sub_base = n.subscribe("stm_publish", 1, speedCallback);
  ros::Subscriber sub_imu = n.subscribe("arduino_jiaodu/Yaw", 1, rotationCallback);
  
  double last_yaw = 0.0;
  
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  
  double v_x = 0.0;
  double v_y = 0.0;
  double v_theta = 0.0;
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while(n.ok())
  {
    ros::spinOnce(); // check for incoming messages
    current_time = ros::Time::now();
    
    // Compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double curr_yaw = yaw_imu_;
    double average_speedx = (front_left - front_right) / 2;
    double average_speedy = (back_right - front_right) / 2;

    double average_distancex = average_speedx * dt;
    double average_distancey = average_speedy * dt;

    double delta_x = average_distancex * cos(curr_yaw) - average_distancey * sin(curr_yaw);
    double delta_y = average_distancex * sin(curr_yaw) + average_distancey * cos(curr_yaw);

    x += delta_x;
    y += delta_y;
    theta = curr_yaw;
    
    v_x = average_speedx;
    v_y = average_speedy;
    v_theta = (curr_yaw - last_yaw) / dt;
    
    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    // First, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
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
    
    // Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    // Set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.angular.z = v_theta;
    
    // Publish the message
    odom_pub.publish(odom);
    last_time = current_time;
    last_yaw = curr_yaw;
    loop_rate.sleep();
  }
  return 0;
}

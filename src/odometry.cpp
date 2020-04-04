#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <grad_project/Float64Stamped.h>
#include <iostream>




class Odometry{
 public:
  Odometry();
 private:
  void OdomCallBack(const  nav_msgs::Odometry::ConstPtr& msg);
  void SpeedCallBack(const grad_project::Float64Stamped::ConstPtr& msg);
  void GyroCallBack(const sensor_msgs::Imu::ConstPtr& msg);
  void PublishOdom();
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub;
  ros::Subscriber speed_sub;
  ros::Subscriber gyro_sub;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Publisher odom_pub;
  nav_msgs::Odometry odom;
};

Odometry::Odometry(){
	
  odom_sub = nh_.subscribe("odom_temp", 1, &Odometry::OdomCallBack, this);
  speed_sub = nh_.subscribe("pioneer3at/gps/speed", 1, &Odometry::SpeedCallBack, this);
  gyro_sub = nh_.subscribe("pioneer3at/gyro/values", 1, &Odometry::GyroCallBack, this);
  odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  
}


void Odometry::OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  //set the position
  odom.pose.pose.position = msg->pose.pose.position;
  odom.pose.pose.orientation = msg->pose.pose.orientation;

  //set the velocity
  //odom.child_frame_id = "base_link";
  PublishOdom();
}


void Odometry::SpeedCallBack(const grad_project::Float64Stamped::ConstPtr& msg){
  odom.twist.twist.linear.x = msg->data;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0; 
  PublishOdom();
}

void Odometry::GyroCallBack(const sensor_msgs::Imu::ConstPtr& msg){
  odom.twist.twist.angular.z = msg->angular_velocity.y * (-1);
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  PublishOdom();
}

void Odometry::PublishOdom(){
  odom_pub.publish(odom);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  Odometry odom;

  ros::spin();
  return 0;
}

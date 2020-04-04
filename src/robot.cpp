// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * To run gmapping you should start gmapping:
 * rosrun gmapping slam_gmapping scan:=/pioneer3at/Sick_LMS_291/laser_scan/layer0 _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30 _delta:=0.2
 */

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

#include <grad_project/set_float.h>
#include <grad_project/set_int.h>

#define TIME_STEP 32
#define NMOTORS 4
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9

ros::NodeHandle *n;

static std::vector<float> lidarValues;

static int controllerCount;
static std::vector<std::string> controllerList;

ros::ServiceClient timeStepClient;
grad_project::set_int timeStepSrv;

static const char *motorNames[NMOTORS] = {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"};

static double GPSValues[3] = {0, 0, 0};
static double inertialUnitValues[4] = {0, 0, 0, 0};

static int lms291Resolution = 0;
static int halfResolution = 0;
static double maxRange = 0.0;
static double rangeThreshold = 0.0;
static std::vector<double> braitenbergCoefficients;
static bool areBraitenbergCoefficientsinitialized = false;

ros::Publisher odom_pub;

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}



void broadcastTransform() {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(-GPSValues[2], GPSValues[0], GPSValues[1]));
  tf::Quaternion q(inertialUnitValues[0], inertialUnitValues[1], inertialUnitValues[2], inertialUnitValues[3]);
  q = q.inverse();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "base_link"));
  transform.setIdentity();
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "pioneer3at/Sick_LMS_291"));

  
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom_temp";

  //set the position
  odom.pose.pose.position.x = GPSValues[2] * (-1);
  odom.pose.pose.position.y = GPSValues[0]; 
  odom.pose.pose.position.z = GPSValues[1];
  odom.pose.pose.orientation.x = inertialUnitValues[0];
  odom.pose.pose.orientation.y = inertialUnitValues[1];
  odom.pose.pose.orientation.z = inertialUnitValues[2] * (-1);
  odom.pose.pose.orientation.w = inertialUnitValues[3];

  //set the velocity
  
  odom_pub.publish(odom);

}

void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values) {
  GPSValues[0] = values->latitude;
  GPSValues[1] = values->altitude;
  GPSValues[2] = values->longitude;
  broadcastTransform();
}

void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {
  inertialUnitValues[0] = values->orientation.x;
  inertialUnitValues[1] = values->orientation.y;
  inertialUnitValues[2] = values->orientation.z;
  inertialUnitValues[3] = values->orientation.w;
  broadcastTransform();
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  int scanSize = scan->ranges.size();
  lidarValues.resize(scanSize);
  for (int i = 0; i < scanSize; ++i)
    lidarValues[i] = scan->ranges[i];

  lms291Resolution = scanSize;
  halfResolution = scanSize / 2;
  maxRange = scan->range_max;
  rangeThreshold = maxRange / 20.0;
  if (!areBraitenbergCoefficientsinitialized) {
    braitenbergCoefficients.resize(lms291Resolution);
    for (int i = 0; i < lms291Resolution; ++i)
      braitenbergCoefficients[i] = gaussian(i, halfResolution, lms291Resolution / 5);
    areBraitenbergCoefficientsinitialized = true;
  }

}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {
  ROS_INFO("User stopped the 'pioneer3at' node.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}




int main(int argc, char **argv) {
  std::string controllerName;
  // create a node named 'pioneer3at' on ROS network
  ros::init(argc, argv, "robot", ros::init_options::AnonymousName);
  n = new ros::NodeHandle;

  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();
  
  odom_pub = n->advertise<nav_msgs::Odometry>("/odom_temp", 1);
  timeStepClient = n->serviceClient<grad_project::set_int>("pioneer3at/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  // if there is more than one controller available, it let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }
  ROS_INFO("Using controller: '%s'", controllerName.c_str());
  // leave topic once it is not necessary anymore
  nameSub.shutdown();

  // init motors
  for (int i = 0; i < NMOTORS; ++i) {
    // position
    ros::ServiceClient set_position_client;
    grad_project::set_float set_position_srv;
    set_position_client = n->serviceClient<grad_project::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    grad_project::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<grad_project::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
  }
  //ros::Subscriber set_vel = n->subscribe("cmd_vel", 10, setVelCallback);

  // enable lidar
  ros::ServiceClient set_lidar_client;
  grad_project::set_int lidar_srv;
  ros::Subscriber sub_lidar_scan;

  set_lidar_client = n->serviceClient<grad_project::set_int>("pioneer3at/Sick_LMS_291/enable");
  lidar_srv.request.value = TIME_STEP;
  if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) {
    ROS_INFO("Lidar enabled.");
    sub_lidar_scan = n->subscribe("pioneer3at/Sick_LMS_291/laser_scan/layer0", 10, lidarCallback);
    ROS_INFO("Topic for lidar initialized.");
    while (sub_lidar_scan.getNumPublishers() == 0) {
    }
    ROS_INFO("Topic for lidar scan connected.");
  } else {
    if (!lidar_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable lidar.");
    return 1;
  }

  // enable gps
  ros::ServiceClient set_GPS_client;
  grad_project::set_int GPS_srv;
  ros::Subscriber sub_GPS;
  set_GPS_client = n->serviceClient<grad_project::set_int>("pioneer3at/gps/enable");
  GPS_srv.request.value = 32;
  if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) {
    
    sub_GPS = n->subscribe("pioneer3at/gps/values", 1, GPSCallback);
    while (sub_GPS.getNumPublishers() == 0) {
    }
    ROS_INFO("GPS enabled.");
  } else {
    if (!GPS_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable GPS.");
    return 1;
  }

  // enable inertial unit
  ros::ServiceClient set_inertial_unit_client;
  grad_project::set_int inertial_unit_srv;
  ros::Subscriber sub_inertial_unit;
  set_inertial_unit_client = n->serviceClient<grad_project::set_int>("pioneer3at/inertial_unit/enable");
  inertial_unit_srv.request.value = 32;
  if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
    sub_inertial_unit = n->subscribe("pioneer3at/inertial_unit/roll_pitch_yaw", 1, inertialUnitCallback);
    while (sub_inertial_unit.getNumPublishers() == 0) {
    }
    ROS_INFO("Inertial unit enabled.");
  } else {
    if (!inertial_unit_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable inertial unit.");
    return 1;
  }

  // enable accelerometer
  ros::ServiceClient set_accelerometer_client;
  grad_project::set_int accelerometer_srv;
  ros::Subscriber sub_accelerometer;
  set_accelerometer_client = n->serviceClient<grad_project::set_int>("pioneer3at/accelerometer/enable");
  accelerometer_srv.request.value = 32;
  set_accelerometer_client.call(accelerometer_srv);
  // enable camera
  ros::ServiceClient set_camera_client;
  grad_project::set_int camera_srv;
  ros::Subscriber sub_camera;
  set_camera_client = n->serviceClient<grad_project::set_int>("pioneer3at/camera/enable");
  camera_srv.request.value = 64;
  set_camera_client.call(camera_srv);
  // enable gyro
  ros::ServiceClient set_gyro_client;
  grad_project::set_int gyro_srv;
  ros::Subscriber sub_gyro;
  set_gyro_client = n->serviceClient<grad_project::set_int>("pioneer3at/gyro/enable");
  gyro_srv.request.value = 32;
  set_gyro_client.call(gyro_srv);

  ROS_INFO("You can now start the creation of the map using 'rosrun gmapping slam_gmapping "
           "scan:=/pioneer3at/Sick_LMS_291/laser_scan/layer0 _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30 _delta:=0.2'.");
  ROS_INFO("You can now visualize the sensors output in rqt using 'rqt'.");

  // main loop
  while (ros::ok()) {
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("Failed to call service time_step for next step.");
      break;
    }
    ros::spinOnce();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::shutdown();
  return 0;
}

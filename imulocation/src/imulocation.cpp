#include "ros/ros.h"
#include "turtlebot3_msgs/SensorState.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>



double t;//当前时间
double latest_time;//上一帧时间

bool init_imu=true;

//估计的位置与姿态
Eigen::Vector3d tmp_P=Eigen::Vector3d(0, 0, 0); //t
Eigen::Quaterniond tmp_Q=Eigen::Quaterniond::Identity();//R
Eigen::Vector3d tmp_V=Eigen::Vector3d(0, 0, 0);

//前一帧的局部坐标系的加速度
Eigen::Vector3d acc_0=Eigen::Vector3d(0, 0, 0);
//前一帧的角速度
Eigen::Vector3d gyr_0=Eigen::Vector3d(0, 0, 0);
//重力加速度


void chatterCallback(const sensor_msgs::ImuConstPtr imu_msg)
{
  
 
  double t = imu_msg->header.stamp.toSec();
  if (init_imu)
  {
      latest_time = t;
      init_imu = 0;
      return;
  }
  double dt = t - latest_time;
  latest_time = t;

  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  Eigen::Vector3d linear_acceleration{dx, dy, dz};

  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;
  Eigen::Vector3d angular_velocity{rx, ry, rz};
  
  Eigen::Vector3d un_acc_0 = tmp_Q * acc_0 ;

  Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity);
  Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd((un_gyr * dt)[0], Eigen::Vector3d::UnitX()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[2], Eigen::Vector3d::UnitZ());

  tmp_Q = tmp_Q * quaternion3;

  Eigen::Vector3d un_acc_1 = tmp_Q * linear_acceleration ;

  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

  tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
  tmp_V = tmp_V + dt * un_acc;

  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
  ROS_INFO("pos:[%f,%f,%f]",tmp_P[0],tmp_P[1],tmp_P[2]);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imulocation");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("imu", 1000, chatterCallback);

  ros::spin();

  return 0;
}
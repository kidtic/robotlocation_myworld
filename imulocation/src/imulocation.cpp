#include "ros/ros.h"
#include "turtlebot3_msgs/SensorState.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

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
  
  //计算当前时刻的姿态
  Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity);
  Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd((un_gyr * dt)[0], Eigen::Vector3d::UnitX()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[2], Eigen::Vector3d::UnitZ());
  tmp_Q = tmp_Q * quaternion3;

  Eigen::Vector3d un_acc = linear_acceleration;

  tmp_V = tmp_V + dt * un_acc;

  Eigen::Vector3d world_velocity=tmp_Q*tmp_V;
  tmp_P=tmp_P+world_velocity*dt;

  gyr_0 = angular_velocity;

  printf("tmp_Q:[%f,%f,%f,%f]\n",tmp_Q.x(),tmp_Q.y(),tmp_Q.z(),tmp_Q.w());
  printf("tmp_V:[%f,%f,%f]\n",tmp_V[0],tmp_V[1],tmp_V[2]);
  printf("world_velocity:[%f,%f,%f]\n",world_velocity[0],world_velocity[1],world_velocity[2]);
  printf("tmp_P:[%f,%f,%f]\n",tmp_P[0],tmp_P[1],tmp_P[2]);
  printf("------/n");
  //printf("pos:[%f,%f,%f]\n",tmp_P[0],tmp_P[1],tmp_P[2]);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_data");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/zbot/imu_data", 1000, chatterCallback);

  ros::spin();

  return 0;
}
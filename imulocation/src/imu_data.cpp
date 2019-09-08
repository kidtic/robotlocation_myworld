#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

ros::Publisher pub;

//方便运算定义一个叉积
geometry_msgs::Vector3 cross(geometry_msgs::Vector3 a,geometry_msgs::Vector3 b);

void chatterCallback(const nav_msgs::OdometryConstPtr msg)
{
  //第一次进入初始化
  static Eigen::Vector3d linear_velocity_last=Eigen::Vector3d(0, 0, 0);
  static double time=msg->header.stamp.toSec();
  static Eigen::Vector3d angular_velocity_last=Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);//上一帧的角速度

  //计算采样间隔
  double dt=msg->header.stamp.toSec() - time;
  //计算imu
  sensor_msgs::Imu imu_data;
  imu_data.header=msg->header;
  imu_data.orientation=msg->pose.pose.orientation;
  imu_data.angular_velocity=msg->twist.twist.angular;

  //-------计算加速度
  double dx = msg->twist.twist.angular.x;
  double dy = msg->twist.twist.angular.y;
  double dz = msg->twist.twist.angular.z;
  Eigen::Vector3d angular_velocity{dx, dy, dz};
  dx = msg->twist.twist.linear.x;
  dy = msg->twist.twist.linear.y;
  dz = msg->twist.twist.linear.z;
  Eigen::Vector3d linear_velocity{dx, dy, dz};
  //计算两帧的旋转rpy以及四元数
  //Eigen::Vector3d un_gyr = 0.5 * (angular_velocity_last + angular_velocity);
  Eigen::Vector3d un_gyr = angular_velocity;
  Eigen::Quaterniond drpyQ;//两帧的旋转欧拉角
    drpyQ = Eigen::AngleAxisd((-un_gyr * dt)[0], Eigen::Vector3d::UnitX()) * 
                  Eigen::AngleAxisd((-un_gyr * dt)[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd((-un_gyr * dt)[2], Eigen::Vector3d::UnitZ());
  
  Eigen::Vector3d linear_acceleration;
  //公式
  linear_acceleration=(linear_velocity-drpyQ*linear_velocity_last)/dt;

  
  imu_data.linear_acceleration.x=linear_acceleration[0];
  imu_data.linear_acceleration.y=linear_acceleration[1];
  imu_data.linear_acceleration.z=linear_acceleration[2];

  pub.publish(imu_data);
  //继承
  time=msg->header.stamp.toSec();
  linear_velocity_last=linear_velocity;
  angular_velocity_last=angular_velocity;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_data");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);
  pub=nh.advertise<sensor_msgs::Imu>("zbot/imu_data",1000);

  ros::spin();

  return 0;
}

geometry_msgs::Vector3 cross(geometry_msgs::Vector3 a,geometry_msgs::Vector3 b)
{
  geometry_msgs::Vector3 ret;
  ret.x=a.y*b.z-b.y*a.z;
  ret.y=a.z*b.x-b.z*a.x;
  ret.z=a.x*b.y-b.x*a.y;
  return ret;
}

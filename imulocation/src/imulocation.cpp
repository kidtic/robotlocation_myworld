#include "ros/ros.h"
#include "turtlebot3_msgs/SensorState.h"
#include "sensor_msgs/Imu.h"

void chatterCallback(const sensor_msgs::ImuPtr msg)
{
  ROS_INFO("imu data：[%f,%f,%f,%f,%f,%f]",msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z,
  msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imulocation");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("imu", 1000, chatterCallback);

  ros::spin();

  return 0;
}
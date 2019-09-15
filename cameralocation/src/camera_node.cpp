/*
* 此为相机模型节点，用于仿真相机传感器像素点的投影。为了节约资源，节省成本，
* 不会对所有像素点成像,只对机器人的3个特征点成像。可以看成是红外摄像头，然后
* 在机器人上安装了感应球。
* 所以发布的传感器数据不是图像，而是3个机器人特征点在图像上的像素坐标
*/

#include <cameralocation/cameradata.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  camera cam(4,4,4,0,0.62,3.93,517,516,325,249);

  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe("/zbot/imu_data", 1000, chatterCallback);
  //zbotOdom=nh.advertise<nav_msgs::Odometry>("zbot/imu_odom",1000);
  //ros::spin();
}
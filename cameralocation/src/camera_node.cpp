/*
* 此为相机模型节点，用于仿真相机传感器像素点的投影。为了节约资源，节省成本，
* 不会对所有像素点成像,只对机器人的3个特征点成像。可以看成是红外摄像头，然后
* 在机器人上安装了感应球。
* 所以发布的传感器数据不是图像，而是3个机器人特征点在图像上的像素坐标
*/

#include <cameralocation/cameradata.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>

using namespace cv;

camera cam;

void chatterCallback(const nav_msgs::OdometryConstPtr msg);


int main(int argc, char **argv)
{
  cam=camera(-4.0, -4.0, 4.0,
  -0.78539815, 0.0 ,-2.186276,
  517.3,516.5,325.1,249.7);

  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;

  //std::cout<<cam.world2pix(Eigen::Vector3d(0,0,0))<<std::endl;

  ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);
  //zbotOdom=nh.advertise<nav_msgs::Odometry>("zbot/imu_odom",1000);
  ros::spin();
  

}

void chatterCallback(const nav_msgs::OdometryConstPtr msg)
{
  Eigen::Vector3d pos=Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
  Eigen::Vector2d pix=cam.world2pix(pos);
  std::cout<<pix<<"\n"<<std::endl;
  
}
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

#define CAMERA_NUM 4


using namespace cv;
using namespace std;



camera cam[CAMERA_NUM];
ros::Publisher pub;

void chatterCallback(const nav_msgs::OdometryConstPtr msg);


int main(int argc, char **argv)
{
  
  
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;

  //input print
  cout<<"argc:"<<argc<<endl;
  for (size_t i = 0; i < argc; i++)
  {
    cout<<argv[i]<<endl;
  }

  //初始化相机
  if(argc==2)//如果有参数
  {
    string configdir=argv[1];
    
    cam[0]=camera(configdir,"camera0");
    cam[1]=camera(configdir,"camera1");
    cam[2]=camera(configdir,"camera2");
    cam[3]=camera(configdir,"camera3");
  }
  else
  {
    string configdir="/home/kk/myproject/ros-projrct/catkin_ws_robotLocation/src/robotlocation_myworld/cameralocation/config/config.yaml";
    
    cam[0]=camera(configdir,"camera0");
    cam[1]=camera(configdir,"camera1");
    cam[2]=camera(configdir,"camera2");
    cam[3]=camera(configdir,"camera3");
  }
  
  

  ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);
  pub=nh.advertise<cameralocation::cameraKeyPoint>("/zbot/camera",1000);
  
  ros::spin();
  

}


//相机刷新率40 imu刷新率120；
void chatterCallback(const nav_msgs::OdometryConstPtr msg)
{
  static double last_time=msg->header.stamp.toSec();
  static int reaptCont=0;//重复计数

  if(reaptCont>=2)//每2次Callback采集一次
  {
    reaptCont=0;
    //计算间隔时间
    double dt=msg->header.stamp.toSec()-last_time;
    //std::cout<<"dt:"<<dt<<std::endl;

    //计算robot位姿
    Eigen::Vector3d pos=Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    Eigen::Quaterniond tmp_Q=Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,
                                                msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    g2o::SE3Quat pos_quat(tmp_Q,pos);
    
    //计算位姿映射到各个相机上的像素点
    Eigen::Matrix<double,2,3> pix;
    cameralocation::cameraKeyPoint robotImgmsg;
    for (size_t i = 0; i < CAMERA_NUM; i++)
    {
      /* code */
      pix=cam[i].robot2pix(pos_quat);
      robotImgmsg.header=msg->header;
      robotImgmsg.org[0+2*i]=(int32_t)pix(0,0);
      robotImgmsg.org[1+2*i]=(int32_t)pix(1,0);
      robotImgmsg.xaxis[0+2*i]=(int32_t)pix(0,1);
      robotImgmsg.xaxis[1+2*i]=(int32_t)pix(1,1);
      robotImgmsg.yaxis[0+2*i]=(int32_t)pix(0,2);
      robotImgmsg.yaxis[1+2*i]=(int32_t)pix(1,2);  
    }
    pub.publish(robotImgmsg);
    //刷新时间
    last_time=msg->header.stamp.toSec();
  }
  else
  {
    
    reaptCont++;
  }
  

}
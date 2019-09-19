#include <ros/ros.h>
#include <cameralocation/location.h>

#define CAMERA_NUM 4

//回调函数
void imuCallback(const sensor_msgs::ImuConstPtr msg);
void cameraCallback0(const cameralocation::cameraKeyPointConstPtr msg);
void cameraCallback1(const cameralocation::cameraKeyPointConstPtr msg);
void cameraCallback2(const cameralocation::cameraKeyPointConstPtr msg);
void cameraCallback3(const cameralocation::cameraKeyPointConstPtr msg);

//定位器
Location location;
//消息订阅
ros::Subscriber imuSub;//订阅了机器人的imu信息
ros::Subscriber cameraSub[CAMERA_NUM];//订阅了相机的消息，相机数量未知。

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "location_node");
    ros::NodeHandle nh;
    location=Location(2);
    //订阅消息
    imuSub=nh.subscribe("/zbot/imu_data",1000,imuCallback);
    cameraSub[0]=nh.subscribe("/zbot/camera0",1000,cameraCallback0);
    cameraSub[1]=nh.subscribe("/zbot/camera1",1000,cameraCallback1);
    cameraSub[2]=nh.subscribe("/zbot/camera2",1000,cameraCallback2);
    cameraSub[3]=nh.subscribe("/zbot/camera3",1000,cameraCallback3);

    ros::spin();
    return 0;
}



//-----------------回调函数
void imuCallback(const sensor_msgs::ImuConstPtr msg)
{
    sensor_msgs::Imu imuinput;
    imuinput.header=msg->header;
    imuinput.angular_velocity=msg->angular_velocity;
    imuinput.linear_acceleration=msg->linear_acceleration;
    imuinput.orientation=msg->orientation;

    location.imuodom.imu_push(imuinput);
    printf("ok\n");

}
void cameraCallback0(const cameralocation::cameraKeyPointConstPtr msg)
{

}
void cameraCallback1(const cameralocation::cameraKeyPointConstPtr msg)
{

}
void cameraCallback2(const cameralocation::cameraKeyPointConstPtr msg)
{

}
void cameraCallback3(const cameralocation::cameraKeyPointConstPtr msg)
{

}
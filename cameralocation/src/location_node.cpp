#include <ros/ros.h>
#include <cameralocation/location.h>

using namespace std;

#define CAMERA_NUM 4

//回调函数
void imuCallback(const sensor_msgs::ImuConstPtr msg);
void cameraCallback(const cameralocation::cameraKeyPointConstPtr msg);


//定位器
Location location;
//消息订阅
ros::Subscriber imuSub;//订阅了机器人的imu信息
ros::Subscriber cameraSub;//订阅了相机的消息，

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "location_node");
    ros::NodeHandle nh;
    //初始化
    if (argc==2)
    {
        location=Location(2,CAMERA_NUM,argv[1]);
    }
    else
    {
        location=Location(2,CAMERA_NUM,"src/robotlocation_myworld/cameralocation/config/config.yaml");
    }
    
    
    
    //订阅消息
    imuSub=nh.subscribe("/zbot/imu_data",1000,imuCallback);
    cameraSub=nh.subscribe("/zbot/camera",1000,cameraCallback);

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
    //printf("ok\n");

}
void cameraCallback(const cameralocation::cameraKeyPointConstPtr msg)
{
    //test 多相机融合定位
    std::vector<Eigen::Vector2d> botPc;
    for (size_t i = 0; i < CAMERA_NUM; i++)
    {
        botPc.push_back(Eigen::Vector2d(msg->org[2*i],msg->org[1+2*i]));
    }
    Eigen::Vector3d robotP=location.MultiCameraLocation(botPc);
    cout<<"robot pose :\n"<<robotP<<endl;
    
}

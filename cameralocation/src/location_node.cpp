#include <ros/ros.h>
#include <cameralocation/location.h>
#include <chrono>

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
    //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    sensor_msgs::Imu imuinput;
    imuinput.header=msg->header;
    imuinput.angular_velocity=msg->angular_velocity;
    imuinput.linear_acceleration=msg->linear_acceleration;
    imuinput.orientation=msg->orientation;
    location.imuodom.updata_robotPQV_fromIMU(imuinput);
    
    //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    //chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    //printf("optimization costs time: %f s",time_used.count());

}
void cameraCallback(const cameralocation::cameraKeyPointConstPtr msg)
{
    //test 多相机融合定位
    /*
    std::vector<Eigen::Vector2d> botPc;
    for (size_t i = 0; i < CAMERA_NUM; i++)
    {
        botPc.push_back(Eigen::Vector2d(msg->org[2*i],msg->org[1+2*i]));
    }
    Eigen::Vector3d robotP=location.MultiCameraLocation(botPc);
    cout<<"robot pose :\n"<<robotP<<endl;
    */



    //------开始新算法验证
    //要做的目的只有一个，这个时候imuodom应该是准备好了的，需要做的只有一个
    //就是根据msg也就是多个相机的robot像素点来更新imuodom.robotPQV[0]

/*
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    //g2o::SE3Quat robotpose;
    //robotpose=location.imuCameraLocation(msg);
    //Imuodom::PQV_type rp=location.imuodom.SE3_to_PQV(robotpose,location.imuodom.robotPQV[0].tmp_V);
    //location.imuodom.robotPQV[0]=rp;
    //location.imuodom.updata_robotPQV_fromIMU();
    //std::cout<<"robotpose:\n"<<location.imuodom.robotPQV.back().tmp_P<<endl;

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;
*/

    //----相机同步+imu补偿算法
    location.camInfoSynchCashe_push(msg);
    //查看相机是否同步--同步ok
    //cout<<location.camInfo[0].robotPixSynch[0](0,0)<<endl;    
    //cout<<location.camInfo[1].robotPixSynch[0](0,0)<<endl;    
    //cout<<location.camInfo[2].robotPixSynch[0](0,0)<<endl;    
    //cout<<location.camInfo[3].robotPixSynch[0](0,0)<<endl;   
     std::vector<Eigen::Matrix<double ,2,3>> camKP_dl;
     for (size_t i = 0; i < location.camInfo.size(); i++)
     {
         camKP_dl.push_back(location.camInfo[i].robotPixSynch[0]);
     }
     g2o::SE3Quat robotPose=location.MultiCameraLocation(camKP_dl);
     cout<<"pose\n"<<robotPose.translation()<<endl;
     cout<<"quat\n"<<robotPose.rotation().w()<<endl;
     

}

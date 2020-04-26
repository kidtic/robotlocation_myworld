#include <ros/ros.h>
#include <cameralocation/location.h>
#include <chrono>
#include <fstream>

using namespace std;

#define CAMERA_NUM 4

//回调函数
void imuCallback(const sensor_msgs::ImuConstPtr msg);
void cameraCallback(const cameralocation::cameraKeyPointConstPtr msg);
void odomCallback(const nav_msgs::OdometryConstPtr msg);



//定位器
Location location;
//消息订阅
ros::Subscriber imuSub;//订阅了机器人的imu信息
ros::Subscriber cameraSub;//订阅了相机的消息，
ros::Subscriber realodom;//订阅机器人的真实位置信息

//发布相机同步定位+imu补偿算法的定位结果
ros::Publisher locationPub;
ros::Publisher onlycameraLocationPub;
ros::Publisher onlyIMULocationPub;

//imu定位的里程
Imuodom imuLocation;
//真实的机器人位置信息
g2o::SE3Quat realRobotLocation;

//用于数据获取
ofstream errorDataFile;
ofstream trackDataFile;

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
    imuLocation=Imuodom(10);
    
    //延时
    //sleep(5);

    errorDataFile.open("LocationErrorData.txt",ios::out);
    trackDataFile.open("LocationTrackData.txt",ios::out);


    //订阅消息
    imuSub=nh.subscribe("/zbot/imu_data",1000,imuCallback);
    cameraSub=nh.subscribe("/zbot/camera",1000,cameraCallback);
    realodom=nh.subscribe("odom",1000,odomCallback);

    //fabu
    locationPub = nh.advertise<geometry_msgs::PoseStamped>("zbot/imucamLocationPose",1000);
    onlycameraLocationPub=nh.advertise<geometry_msgs::PoseStamped>("zbot/camLocationPose",1000);
    onlyIMULocationPub=nh.advertise<geometry_msgs::PoseStamped>("zbot/onlyimuLocationPose",1000);


    ros::spin();
    errorDataFile.close();
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

    //IMU定位估计
    imuLocation.updata_robotPQV_fromIMU(imuinput);
    
    //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    //chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    //printf("optimization costs time: %f s",time_used.count());

}
void cameraCallback(const cameralocation::cameraKeyPointConstPtr msg)
{
    static Imuodom::PQV_type lastPQV;
    static double lastRosTime=msg->header.stamp.toSec();

    double errordata[3];//定位误差
    double trackdata[10]; //三个定位方法的轨迹图

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

    //---------------相机同步+imu补偿算法
    location.camInfoSynchCashe_push(msg);
    //相机同步   
    std::vector<Eigen::Matrix<double ,2,3>> camKP_dl;
    for (size_t i = 0; i < location.camInfo.size(); i++)
    {
        camKP_dl.push_back(location.camInfo[i].robotPixSynch[0]);
    }
    //相机定位
    g2o::SE3Quat robotPose=location.MultiCameraLocation(camKP_dl);
    //通过相机定位计算相机定位速度。
    double dt=msg->header.stamp.toSec()-lastRosTime;
    if (dt>0.03||dt<0.02)dt=0.025;
    Eigen::Vector3d robotV_cam=(robotPose.translation()-lastPQV.tmp_P)/dt;

    //imu补偿
    Imuodom::PQV_type firstPQV=Imuodom::SE3_to_PQV(robotPose,0.05*robotV_cam+0.95*location.imuodom.robotPQV[0].tmp_V);
    //Imuodom::PQV_type firstPQV=Imuodom::SE3_to_PQV(robotPose,location.imuodom.robotPQV[0].tmp_V);
    //Imuodom::PQV_type firstPQV=Imuodom::SE3_to_PQV(robotPose,robotV_cam);
    location.imuodom.robotPQV[0]=firstPQV;
    location.imuodom.updata_robotPQV_fromIMU();

    double x=location.imuodom.robotPQV.back().tmp_P[0];
    double y=location.imuodom.robotPQV.back().tmp_P[1];
    double z=location.imuodom.robotPQV.back().tmp_P[2];
    cout<<"robotpose:\n"<<(int)(x*100)/100.0<<"\n"<<(int)(y*100)/100.0<<"\n"<<(int)(z*100)/100.0<<"\n"<<endl;
    cout<<"robotquat:\n"<<location.imuodom.robotPQV.back().tmp_Q.w()<<endl;
    //如果定位故障，则停止
    if((abs(x)+abs(y)+abs(z))>30||abs((int)(x*100)/100.0)>10||abs(y)>10)
    { 
        cout<<"定位故障"<<endl;
        cout<<"firstPQV.tmp_p:\n"<<firstPQV.tmp_P<<endl;
        cout<<"firstPQV.tmp_q:\n"<<firstPQV.tmp_Q.w()<<","<<
            firstPQV.tmp_Q.x()<<","<<
            firstPQV.tmp_Q.y()<<","<<
            firstPQV.tmp_Q.z()<<endl;
        cout<<"firstPQV.tmp_v:\n"<<firstPQV.tmp_V<<endl;
        for (size_t i = 0; i < location.imuodom.robotPQV.size(); i++)
        {
            cout<<"robotPQV["<<i<<"]:\n"<<location.imuodom.robotPQV[i].tmp_P<<endl;
        }
        
        ros::shutdown();
    }
     
    geometry_msgs::PoseStamped robotposeOut;
    robotposeOut.header=msg->header;
    robotposeOut.pose.position.x=x;
    robotposeOut.pose.position.y=y;
    robotposeOut.pose.position.z=z;
    robotposeOut.pose.orientation.x=location.imuodom.robotPQV.back().tmp_Q.x();
    robotposeOut.pose.orientation.y=location.imuodom.robotPQV.back().tmp_Q.y();
    robotposeOut.pose.orientation.z=location.imuodom.robotPQV.back().tmp_Q.z();
    robotposeOut.pose.orientation.w=location.imuodom.robotPQV.back().tmp_Q.w();
    locationPub.publish(robotposeOut);
    errordata[0] = (realRobotLocation.translation()-location.imuodom.robotPQV.back().tmp_P).norm();

    //---------------相机融合定位
    std::vector<Eigen::Matrix<double ,2,3>> camKP_dl0;
    for (size_t i = 0; i < location.camInfo.size(); i++)
    {
        camKP_dl0.push_back(location.camInfo[i].robotPixSynch.back());
    }
    //相机定位
    robotPose=location.MultiCameraLocation(camKP_dl0);

    geometry_msgs::PoseStamped robotposeOut1;
    robotposeOut1.header=msg->header;
    robotposeOut1.pose.position.x=robotPose.translation()[0];
    robotposeOut1.pose.position.y=robotPose.translation()[1];
    robotposeOut1.pose.position.z=robotPose.translation()[2];
    robotposeOut1.pose.orientation.x=robotPose.rotation().x();
    robotposeOut1.pose.orientation.y=robotPose.rotation().y();
    robotposeOut1.pose.orientation.z=robotPose.rotation().z();
    robotposeOut1.pose.orientation.w=robotPose.rotation().w();
    onlycameraLocationPub.publish(robotposeOut1);
    errordata[1] = (realRobotLocation.translation()-robotPose.translation()).norm();

    //---------------IMU里程计
    robotPose=Imuodom::PQV_to_SE3(imuLocation.robotPQV.back());
    geometry_msgs::PoseStamped robotposeOut2;
    robotposeOut2.header=msg->header;
    robotposeOut2.pose.position.x=robotPose.translation()[0];
    robotposeOut2.pose.position.y=robotPose.translation()[1];
    robotposeOut2.pose.position.z=robotPose.translation()[2];
    robotposeOut2.pose.orientation.x=robotPose.rotation().x();
    robotposeOut2.pose.orientation.y=robotPose.rotation().y();
    robotposeOut2.pose.orientation.z=robotPose.rotation().z();
    robotposeOut2.pose.orientation.w=robotPose.rotation().w();
    onlyIMULocationPub.publish(robotposeOut2);
    errordata[2] = (realRobotLocation.translation()-robotPose.translation()).norm();

    lastPQV=firstPQV;
    lastRosTime=msg->header.stamp.toSec();

    //-------------数据记录
    //计算误差写数据
    if(msg->header.stamp.toSec()>6.0)
    {
        errorDataFile<<msg->header.stamp.toSec() <<" " 
                    <<errordata[0]<<" "
                    <<errordata[1]<<" "
                    <<errordata[2]<<" "
                    << endl;
    }
    //计算轨迹
    if(msg->header.stamp.toSec()>6.0)
    {
        trackDataFile<<msg->header.stamp.toSec() <<" " 
                    <<robotposeOut.pose.position.x<<" "
                    <<robotposeOut.pose.position.y<<" "
                    <<robotposeOut.pose.position.z<<" "
                    <<robotposeOut1.pose.position.x<<" "
                    <<robotposeOut1.pose.position.y<<" "
                    <<robotposeOut1.pose.position.z<<" "
                    <<robotposeOut2.pose.position.x<<" "
                    <<robotposeOut2.pose.position.y<<" "
                    <<robotposeOut2.pose.position.z<<" "
                    << endl;
    }

}

void odomCallback(const nav_msgs::OdometryConstPtr msg)
{
    Eigen::Quaterniond realquat(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    Eigen::Vector3d pose(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    realRobotLocation=g2o::SE3Quat(realquat,pose);
}

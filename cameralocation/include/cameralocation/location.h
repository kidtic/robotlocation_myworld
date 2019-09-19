#include <cameralocation/cameraKeyPoint.h>
#include <cameralocation/cameradata.h>
#include <cameralocation/imuodom.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/Imu.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <stdio.h>
#include <ros/ros.h>
#include <string.h>

class Location
{
private:
    /* data */
    std::vector<camera> camInfo;//与cameraSub对应的相机
    
public://data
    
    Imuodom imuodom;//imu里程计，记录imu数据
public://func
    Location();
    Location(int casheSec,int camNum,std::string configpath);//casheSec:存储多少秒内的数据.camNUM:有多少相机
    ~Location();

    /*
    * 利用多相机线性融合算法计算机器人的中心点坐标
    * 参数：Pc：相机对应的机器人像素坐标点
    * 
    * 
    */ 
    Eigen::Vector3d MultiCameraLocation(std::vector<Eigen::Vector2d> Pc);

    
    
};

Location::Location()
{

}



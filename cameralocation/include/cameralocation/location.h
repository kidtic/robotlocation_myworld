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

class Location
{
private:
    /* data */
    std::vector<camera> camInfo;//与cameraSub对应的相机
    
public://data
    
    Imuodom imuodom;//imu里程计，记录imu数据
public://func
    Location();
    Location(int casheSec);//casheSec:存储多少秒内的数据
    ~Location();

  
    
};

Location::Location()
{

}



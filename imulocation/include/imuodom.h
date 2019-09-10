#include "ros/ros.h"
#include "turtlebot3_msgs/SensorState.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "nav_msgs/Odometry.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>


class Imuodom
{
    
private://data
    //最近3s内的缓存的IMU的数据队列，可以查看最近的任意时刻的IMU数据
    std::vector<sensor_msgs::Imu> imudata;
  
    /* data */
public://data
    //订阅的imu数据
    ros::Subscriber imu_topic; 
    //最近3s内的机器人的位置姿态速度
    struct PQV_type
    {
        Eigen::Vector3d tmp_P;
        Eigen::Quaterniond tmp_Q;
        Eigen::Vector3d tmp_V;
    };
    std::vector<PQV_type> robotPQV;



public://func
    Imuodom(/* args */);
    ~Imuodom();

    /*
    * 说明：操作imudata，押入一个最新的数据，并且丢弃最末尾的数据
    * 参数：input：输入的IMU数据
    * ok
    */
    void imu_push(sensor_msgs::Imu input);
    /*
    * 说明：初始化imudata，并且说明缓存队列的最大队列数
    * 参数：maxQueueNum：最大的队列数，缓存容量
    * ok
    */
    void imu_init(int maxQueueNum);
    /*
    * 说明：清除所有队列，值全部清0
    * ok
    */ 
    void imu_clear();

    /*
    * 说明：从最新如入队的顺序来引索
    * 参数：index：引索
    * 返回：IMU数据
    * ok
    */
    sensor_msgs::Imu imu_data_new(int index);

    /*
    * 说明：从最后进队的顺序来引索
    * 参数：index：引索
    * 返回：IMU数据
    * ok
    */
    sensor_msgs::Imu imu_data(int index);

    /*
    * 说明：返回imu队列大小
    * ok
    */
    int imu_size();

    /*
    * 说明：提供IMU运动模型的计算方法,IMU运动方程
    * 参数：last_pqv:上一个时刻的，机器人的状态
    *      imuinput:这一时刻的IMU数据
    *      dt:时间间隔
    * 返回:下一时刻的机器人状态
    * ok
    */
    PQV_type  imu_motion_function(PQV_type last_pqv,sensor_msgs::Imu imuinput,double dt);
    //
};


Imuodom::~Imuodom()
{
}



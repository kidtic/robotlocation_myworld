/*
* imuodom.h
* imu里程计：用于存储一段时间内的imu数据缓存，并且提供IMU里程计算的支持与缓存
* 
*/


#include "ros/ros.h"
#include "turtlebot3_msgs/SensorState.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <g2o/types/sba/types_six_dof_expmap.h>



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
        Eigen::Vector3d tmp_V;//世界坐标系下的速度
    };
    std::vector<PQV_type> robotPQV;
    //debug
    std::vector<double> imudata_dt;//时间间隔对应这每一个dt

public://func
    Imuodom();
    Imuodom(int maxQueueNum);
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


    /*
    * 说明：初始化robotPQV
    * 参数：maxQueueNum：最大的队列数，缓存容量
    * ok
    */ 
    void robotPQV_init(int maxQueueNum);


    /*
    * 说明：押入一个最新的数据，并且丢弃最末尾的数据
    * 
    */ 
    void robotPQV_push(PQV_type input);

    /*
    * 说明：SE3与PQV_type的转换，由于SE3与PQV_type差了一个速度，所
    *      以得补一个速度
    * ok
    */ 
    g2o::SE3Quat PQV_to_SE3(PQV_type input);
    PQV_type SE3_to_PQV(g2o::SE3Quat input,Eigen::Vector3d tmp_V);

    /*
    * 说明：根据imudata，更新robotPQV
    * 
    * 
    */
    void updata_robotPQV_fromIMU(); 

    /*
    * 说明：新得到的imudata值，更新robotPQV，并且将新的imudata push进队列
    * 
    * 
    */
    void updata_robotPQV_fromIMU(sensor_msgs::Imu input); 

    /*
    * 说明：用在g2o优化上的一个函数，可以根据最旧的那个位姿以及IMU数据往前
    *      迭代n次，得到一个比较新的位姿。其中最旧的那个位姿会作为优化变量
    * 参数：n：迭代多少次
    *      se3：最开始的那个位姿
    *      V0：最开始机器人的速度
    * 返回：根据IMU数据迭代n次后的结果。
    * ok
    */ 
    g2o::SE3Quat FB(int n ,g2o::SE3Quat se3,Eigen::Vector3d V0);
};

Imuodom::Imuodom()
{
}
Imuodom::~Imuodom()
{
}



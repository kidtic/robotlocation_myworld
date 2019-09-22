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
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

// g2o edge
class EdgeIMUCameraLocationPoseOnly : public g2o::BaseUnaryEdge<6, Eigen::Matrix<double,6,1>, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeIMUCameraLocationPoseOnly( const camera& cam, const Imuodom imu ) : _cam(cam) ,_imuodom(imu) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        //_error = _measurement - pose->estimate().map( _point );
        int delayn=_imuodom.imu_size() - _cam.robotpix_size()*3;
        Eigen::Matrix<double,2,3> fx=_cam.robot2pix(_imuodom.FB(delayn, pose->estimate(), _imuodom.robotPQV[0].tmp_V));
        Eigen::Matrix<double,6,1> haha;
        haha<<fx(0,0),fx(1,0),fx(0,1),fx(1,1),fx(0,2),fx(1,2);

        _error = _measurement - haha;
    }
    
    

    bool read ( std::istream& in ) {}
    bool write ( std::ostream& out ) const {}
protected:
    camera _cam;
    Imuodom _imuodom;
};

//定位器
class Location
{
private:
    /* data */
    
    
public://data
    
    Imuodom imuodom;//imu里程计，记录imu数据
    std::vector<camera> camInfo;//与cameraSub对应的相机
public://func
    Location();
    Location(int casheSec,int camNum,std::string configpath);//casheSec:存储多少秒内的数据.camNUM:有多少相机
    ~Location();

    /*
    * camera 的同步缓存初始化
    * 会根据camInfo的延时大小来初始化缓存大小。
    */ 
    void cameraCashe_init();

    /*
    * 将相机的robot像素点数据push进对应相机的同步缓存。
    * 
    */ 
    void camInfoSynchCashe_push(int cameraIndex,Eigen::Matrix<double,2,3> inout);
    void camInfoSynchCashe_push( cameralocation::cameraKeyPointConstPtr msg);

    /*
    * 利用多相机线性融合算法计算机器人的中心点坐标
    * 参数：Pc：相机对应的机器人像素坐标点
    * 
    */ 
    Eigen::Vector3d MultiCameraLocation(std::vector<Eigen::Vector2d> Pc);
    /*
    * 利用多相机线性融合算法计算机器人的位置姿态
    * 参数：camKP：每一个相机 捕捉到的机器人的3关键点的像素坐标投影
    */ 
    g2o::SE3Quat MultiCameraLocation(std::vector<Eigen::Matrix<double ,2,3>> camKP);

    /*
    * 新的算法定位，主要靠imu先实时记录近时间段的轨迹，再通过相机来矫正
    * IMU的轨迹。
    * 参数：Pc：相机数据
    *    ：imuoddata：imu里程计
    * 
    */
    g2o::SE3Quat imuCameraLocation(const cameralocation::cameraKeyPointConstPtr msg);

    
    
};

Location::Location()
{

}



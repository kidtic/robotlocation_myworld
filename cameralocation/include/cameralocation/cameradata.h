/*
* 相机模型库：提供相机模型的对象
*
*/
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "iostream"
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <string.h>
#include <ros/ros.h>

#include <cameralocation/cameraKeyPoint.h>



class camera
{
private:
    

public://data
    /* data */
    Eigen::Matrix3d Mi;
    Eigen::Matrix4d Tran;
    //为了简化计算的一些data
    Eigen::Vector3d t;//平移矩阵
    Eigen::Matrix3d R;//旋转矩阵
    Eigen::Matrix3d RM;//可以与Pc组成K向量
    Eigen::Vector3d RT;//B向量

public://func
    camera(std::string path,std::string cameraName);
    camera();
    //世界坐标转换像素坐标
    Eigen::Vector2d world2pix(Eigen::Vector3d pos);
    //世界坐标转换成带深度信息的像素坐标
    Eigen::Vector3d world2pixdep(Eigen::Vector3d pos);

    //带上方向，显示机器人的x轴与y轴，在相机中以3个像素点的信息
    Eigen::Matrix<double,2,3> robot2pix(g2o::SE3Quat input);

};




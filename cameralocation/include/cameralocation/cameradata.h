/*
* 相机模型库：提供相机模型的对象
*
*/
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "iostream"



class camera
{
private:
    /* data */
    Eigen::Matrix3d Mi;
    Eigen::Matrix4d Tran;

public://data

public://func
    camera(double x,double y,double z,
    double roll ,double pitch ,double yaw ,
    double fx,double fy,double cx,double cy);

    //世界坐标转换像素坐标
    Eigen::Vector2d world2pix(Eigen::Vector3d pos);
    //世界坐标转换成带深度信息的像素坐标
    Eigen::Vector3d world2pixdep(Eigen::Vector3d pos);

};



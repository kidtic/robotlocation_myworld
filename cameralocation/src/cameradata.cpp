#include <cameralocation/cameradata.h>

camera::camera(double x,double y,double z,
    double roll ,double pitch ,double yaw ,
    double fx,double fy,double cx,double cy)
{
    Mi<<fx,0,cx,
    0,fy,cy,
    0,0,1;

    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * 
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d pos ;
    pos << x, y, z;
    Tran<<rotation_matrix3.transpose(),-pos,0,0,0,1;
    std::cout<<Tran<<std::endl;
}
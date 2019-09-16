#include <cameralocation/cameradata.h>


camera::camera(double x,double y,double z,
    double roll ,double pitch ,double yaw ,
    double fx,double fy,double cx,double cy)
{
    Mi<<fx,0,cx,
    0,fy,cy,
    0,0,1;

    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitX())).toRotationMatrix();
    Eigen::Vector3d pos ;
    pos << x, y, z;
    //求SE3的逆
    Tran<<rotation_matrix3.transpose(),-rotation_matrix3.transpose()*pos,0,0,0,1;
    
    std::cout<<"初始化成功：T=\n"<<Tran<<std::endl;
}
camera::camera(){}

Eigen::Vector2d camera::world2pix(Eigen::Vector3d pos)
{
    Eigen::Vector4d posi;
    posi<<pos,1;
    Eigen::Matrix<double,3,4> Mio;
    Mio<<Mi,Eigen::Vector3d(0,0,0);
    Eigen::Vector3d z=Mio*(Tran*posi);
    //Eigen::Vector3d z=Eigen::Vector3d(0,0,1);
    //std::cout<<"campos=\n"<<(Tran*posi)<<std::endl;
    //std::cout<<"z=\n"<<z<<std::endl;
    Eigen::Vector2d ret;
    ret[0]=z[0]/z[2];ret[1]=z[1]/z[2];
    return ret;
}
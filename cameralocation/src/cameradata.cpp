#include <cameralocation/cameradata.h>


camera::camera(std::string path,std::string cameraName,ros::NodeHandle nh)
{
   
    
    YAML::Node config=YAML::LoadFile(path);
    printf("ok\n");
    double x=config[cameraName+".x"].as<double>();
    double y=config[cameraName+".y"].as<double>();
    double z=config[cameraName+".z"].as<double>();
    double roll=config[cameraName+".roll"].as<double>();
    double pitch=config[cameraName+".pitch"].as<double>();
    double yaw=config[cameraName+".yaw"].as<double>();
    double fx=config[cameraName+".fx"].as<double>();
    double fy=config[cameraName+".fy"].as<double>();
    double cx=config[cameraName+".cx"].as<double>();
    double cy=config[cameraName+".cy"].as<double>();

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
    //初始化topic
    pub=nh.advertise<cameralocation::cameraKeyPoint>("zbot/"+cameraName,1000);
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

//世界坐标转换成带深度信息的像素坐标
Eigen::Vector3d camera::world2pixdep(Eigen::Vector3d pos)
{
    Eigen::Vector4d posi;
    posi<<pos,1;
    Eigen::Matrix<double,3,4> Mio;
    Mio<<Mi,Eigen::Vector3d(0,0,0);
    Eigen::Vector3d z=Mio*(Tran*posi);
    return z;
}

Eigen::Matrix<double,2,3> camera::robot2pix(g2o::SE3Quat input)
{
    Eigen::Matrix<double,2,3> ret;
    Eigen::Vector2d z[3];
    z[0]=world2pix(input*g2o::Vector3(0,0,0));
    z[1]=world2pix(input*g2o::Vector3(0.1,0,0));
    z[2]=world2pix(input*g2o::Vector3(0,0.1,0));
    ret<<z[0],z[1],z[2];
    return ret;
}
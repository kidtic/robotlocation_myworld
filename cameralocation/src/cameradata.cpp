#include <cameralocation/cameradata.h>


camera::camera(std::string path,std::string cameraName)
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
    double delay=config[cameraName+".delay"].as<double>();

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

    //计算优化数据
    R=rotation_matrix3.transpose();
    t=-rotation_matrix3.transpose()*pos;
    RM=R.inverse()*Mi.inverse();
    RT=R.inverse()*t;
    //初始化robotpix缓存。根据相机延时时间进行缓存大小设置
    int delayNum=CAMERA_FPS*delay;
    for (size_t i = 0; i < delayNum; i++)
    {
        Eigen::Matrix<double,2,3> apix;
        robotpix.push_back(apix);
        if(robotpix.size()==delayNum)
        {
            break;
        }
        else if(robotpix.size()>delayNum)
        {
            printf("error:robotpix datasize is too big\n");
            break;
        }
    }
    
    
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
   
    z[0]=world2pix(input*g2o::Vector3D(0,0,0));
    z[1]=world2pix(input*g2o::Vector3D(0.1,0,0));
    z[2]=world2pix(input*g2o::Vector3D(0,0.1,0));
    ret<<z[0],z[1],z[2];
    return ret;
}

void camera::robotpix_push(Eigen::Matrix<double,2,3> input)
{
    robotpix.push_back(input);
    std::vector<Eigen::Matrix<double,2,3>>::iterator e=robotpix.begin();
    robotpix.erase(e);
}

//返回robotpix数据
Eigen::Matrix<double,2,3> camera::robotpix_Index(int index)
{
    return robotpix[index];
}

//返回robotpix的大小
int camera::robotpix_size()
{
    return robotpix.size();
}

int camera::delay_size()
{
    return robotpix.size();
}


Eigen::Matrix<double,2,3> camera::msg2mat(cameralocation::cameraKeyPointConstPtr msg,int cameraIndex)
{
    //解析msg
    Eigen::Matrix<double,2,3> camKP;
    
    camKP<< msg->org[2*cameraIndex],  msg->xaxis[2*cameraIndex], msg->yaxis[2*cameraIndex], 
            msg->org[1+2*cameraIndex],msg->xaxis[1+2*cameraIndex], msg->yaxis[1+2*cameraIndex];
    return camKP;
}
std::vector<Eigen::Matrix<double,2,3>> camera::msg2mat(cameralocation::cameraKeyPointConstPtr msg)
{
    //解析msg
    std::vector< Eigen::Matrix<double,2,3> > camKP;
    int cameraNUm=msg->org.size()/2;
    for (size_t i = 0; i < cameraNUm; i++)
    {
        Eigen::Matrix<double,2,3> pa;
        pa <<   msg->org[2*i],   msg->xaxis[2*i],  msg->yaxis[2*i], 
                msg->org[1+2*i],msg->xaxis[1+2*i],msg->yaxis[1+2*i];
        camKP.push_back(pa);  
    }
    return camKP;
}
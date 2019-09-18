#include <cameralocation/imuodom.h>



//----------------------IMUodom class-------------------
 
void Imuodom::imu_init(int maxQueueNum)
{
  for (size_t i = 0; i < maxQueueNum; i++)
  {
    /* code */
    sensor_msgs::Imu input;
    imudata.push_back(input);
    imudata_dt.push_back(0);
    //printf("%d\n",imudata.size());
    if(imudata.size()==maxQueueNum)
    {
      break;
    }
    else if(imudata.size()>maxQueueNum)
    {
      printf("error:imu_init datasize is too big\n");
      break;
    }
  }
  if(imudata_dt.size()!=imudata.size())
    printf("error: imudata_dt.size()!=imudata.size()");
  
}

sensor_msgs::Imu Imuodom::imu_data(int index)
{

  return imudata[index];
}

sensor_msgs::Imu Imuodom::imu_data_new(int index)
{
  return imudata[imudata.size()-index-1];
}

void Imuodom::imu_push(sensor_msgs::Imu input)
{
  //采样间隔
  double t=input.header.stamp.toSec()-imudata.back().header.stamp.toSec();
  //容错
  if (t>0.5) t=0;
  //push
  imudata.push_back(input);
  std::vector<sensor_msgs::Imu>::iterator e=imudata.begin();
  imudata.erase(e);
  imudata_dt.push_back(t);
  std::vector<double>::iterator et=imudata_dt.begin();
  imudata_dt.erase(et);

}
int Imuodom::imu_size()
{
  return imudata.size();
}


Imuodom::Imuodom(int maxQueueNum)
{
  imu_init(maxQueueNum);
  robotPQV_init(maxQueueNum+1);
}

void Imuodom::imu_clear()
{
  for (size_t i = 0; i < imudata.size(); i++)
  {
    /* code */
    imudata[i].linear_acceleration.x=0;
    imudata[i].linear_acceleration.y=0;
    imudata[i].linear_acceleration.z=0;
    imudata[i].angular_velocity.x=0;    
    imudata[i].angular_velocity.y=0;
    imudata[i].angular_velocity.z=0;
    imudata[i].orientation.w=0;
    imudata[i].orientation.x=0;
    imudata[i].orientation.y=0;
    imudata[i].orientation.z=0;
  }
  
}

Imuodom::PQV_type  Imuodom::imu_motion_function(PQV_type last_pqv,sensor_msgs::Imu imuinput,double dt)
{
  double dx = imuinput.linear_acceleration.x;
  double dy = imuinput.linear_acceleration.y;
  double dz = imuinput.linear_acceleration.z;
  Eigen::Vector3d linear_acceleration{dx, dy, dz};

  double rx = imuinput.angular_velocity.x;
  double ry = imuinput.angular_velocity.y;
  double rz = imuinput.angular_velocity.z;
  Eigen::Vector3d angular_velocity{rx, ry, rz};

  PQV_type ret;

//计算当前时刻的姿态
  Eigen::Vector3d un_gyr =  angular_velocity;
  Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd((un_gyr * dt)[0], Eigen::Vector3d::UnitX()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[2], Eigen::Vector3d::UnitZ());
  ret.tmp_Q = last_pqv.tmp_Q * quaternion3;
  Eigen::Vector3d un_acc = ret.tmp_Q*((linear_acceleration));
  ret.tmp_P=last_pqv.tmp_P+last_pqv.tmp_V*dt+0.5*dt*dt*un_acc;
  ret.tmp_V = last_pqv.tmp_V + dt * un_acc;

  return ret;
}


void Imuodom::robotPQV_init(int maxQueueNum)
{
  for (size_t i = 0; i < maxQueueNum; i++)
  {
    /* code */
    PQV_type input;
    input.tmp_P=Eigen::Vector3d(0, 0, 0); //t
    input.tmp_Q=Eigen::Quaterniond::Identity();//R
    input.tmp_V=Eigen::Vector3d(0, 0, 0);
    robotPQV.push_back(input);
    //printf("%d\n",robotPQV.size());
    if(robotPQV.size()==maxQueueNum)
    {
      break;
    }
    else if(robotPQV.size()>maxQueueNum)
    {
      printf("error:robotPQV_init datasize is too big\n");
      break;
    }
  }
}

void Imuodom::robotPQV_push(PQV_type input)
{
  robotPQV.push_back(input);
  std::vector<Imuodom::PQV_type>::iterator e=robotPQV.begin();
  robotPQV.erase(e);
}

g2o::SE3Quat Imuodom::PQV_to_SE3(PQV_type input)
{
  return g2o::SE3Quat(input.tmp_Q,input.tmp_P);
}

Imuodom::PQV_type Imuodom::SE3_to_PQV(g2o::SE3Quat input,Eigen::Vector3d tmp_V)
{
  PQV_type ret;
  ret.tmp_P=input.translation();
  ret.tmp_Q=input.rotation();
  ret.tmp_V=tmp_V;
  return ret;

}

void Imuodom::updata_robotPQV_fromIMU()
{
  for (size_t i = 0; i < imudata.size(); i++)
  {
    /* code */
    robotPQV[i+1]=imu_motion_function(robotPQV[i],imudata[i],imudata_dt[i]);
  }
  
}

void Imuodom::updata_robotPQV_fromIMU(sensor_msgs::Imu input)
{
  PQV_type pose;
  double dddt=input.header.stamp.toSec()-imudata.back().header.stamp.toSec();
  pose=imu_motion_function(robotPQV.back(),input,dddt);
  robotPQV_push(pose);
  imu_push(input);
  
}

g2o::SE3Quat Imuodom::FB(int n ,g2o::SE3Quat se3,Eigen::Vector3d V0)
{
  PQV_type pose0=SE3_to_PQV(se3,V0);
  for (size_t i = 0; i < n; i++)
  {
    /* code */
    pose0=imu_motion_function(pose0,imudata[i],imudata_dt[i]);
  }
  g2o::SE3Quat ret=PQV_to_SE3(pose0);
  return ret;
  
}
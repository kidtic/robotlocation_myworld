#include <imuodom.h>


ros::Publisher zbotOdom;

double t;//当前时间
double latest_time;//上一帧时间

bool init_imu=true;
 
//估计的位置与姿态
Eigen::Vector3d tmp_P=Eigen::Vector3d(0, 0, 0); //t
Eigen::Quaterniond tmp_Q=Eigen::Quaterniond::Identity();//R
Eigen::Vector3d tmp_V=Eigen::Vector3d(0, 0, 0);



void chatterCallback(const sensor_msgs::ImuConstPtr imu_msg)
{
  
 
  double t = imu_msg->header.stamp.toSec();
  if (init_imu)
  {
      latest_time = t;
      init_imu = 0;
      return;
  }
  double dt = t - latest_time;
  latest_time = t;

  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  Eigen::Vector3d linear_acceleration{dx, dy, dz};

  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;
  Eigen::Vector3d angular_velocity{rx, ry, rz};
  
  //计算当前时刻的姿态
  Eigen::Vector3d un_gyr =  angular_velocity;
  Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd((un_gyr * dt)[0], Eigen::Vector3d::UnitX()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd((un_gyr * dt)[2], Eigen::Vector3d::UnitZ());
  tmp_Q = tmp_Q * quaternion3;
  Eigen::Vector3d un_acc = tmp_Q*((linear_acceleration));
  tmp_P=tmp_P+tmp_V*dt+0.5*dt*dt*un_acc;
  tmp_V = tmp_V + dt * un_acc;

  //发布IMU里程计
  nav_msgs::Odometry odom;
  odom.header=imu_msg->header;
  odom.child_frame_id="base_footprint";
  odom.pose.pose.position.x=tmp_P[0];
  odom.pose.pose.position.y=tmp_P[1];
  odom.pose.pose.position.z=tmp_P[2];
  odom.pose.pose.orientation.x=tmp_Q.x();
  odom.pose.pose.orientation.y=tmp_Q.y();
  odom.pose.pose.orientation.z=tmp_Q.z();
  odom.pose.pose.orientation.w=tmp_Q.w();

  zbotOdom.publish(odom);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imulocation");
  ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe("/zbot/imu_data", 1000, chatterCallback);
  //zbotOdom=nh.advertise<nav_msgs::Odometry>("zbot/imu_odom",1000);
  //ros::spin();
  Imuodom imuod;
  printf("size:%d\n",imuod.imu_size());



  return 0;
}



//IMUodom class
 
void Imuodom::imu_init(int maxQueueNum)
{
  for (size_t i = 0; i < maxQueueNum; i++)
  {
    /* code */
    sensor_msgs::Imu input;
    imudata.push_back(input);
    printf("%d\n",imudata.size());
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
  imudata.push_back(input);
  std::vector<sensor_msgs::Imu>::iterator e=imudata.begin();
  imudata.erase(e);
}
int Imuodom::imu_size()
{
  return imudata.size();
}


Imuodom::Imuodom(/* args */)
{
  imu_init(20);
}
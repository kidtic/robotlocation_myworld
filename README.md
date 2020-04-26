# 步骤
## 准备
1. 首先按照[这个博客](https://www.jianshu.com/p/014552bcc04c),来搭建turtlebot3的仿真环境。在自己的catkinwk/src中

2. 需要改变turtlebot3的里程计参数，在`turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro`中找到插件:
```xml
<plugin name="turtlebot3_waffle_controller" filename="libgazebo_ros_diff_drive.so">
```
将下面的`<updateRate>`由原来的30改为120

## 依赖

安装依赖：`g2o`、`opencv`,  g2o的依赖还需要装eigen3，官方有说到。

(https://github.com/RainerKuemmerle/g2o)

(https://github.com/opencv/opencv)

## 下载克隆安装
1. `git clone`这个包

2. `catkin_make`

# 记录
相机刷新率40Hz，IMU刷新率120Hz

# 效果
运行
``` shell
roslaunch cameralocation cameralocation_gazebo.launch
```
出现gazebo和rviz,rviz显示了三种定位算法的pose箭头，其中天蓝色箭头表示imu姿态估计的pose，蓝色箭头表示通过4个有延时的相机进行定位的结果，红色是融合imu与相机的定位结果。
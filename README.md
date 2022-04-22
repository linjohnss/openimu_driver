# openimu_driver
A simple OpenIMU driver for ROS

## Build Package
```shell
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/linjohnss/openimu_driver.git
catkin_make
```
## Run openrtk_ndoe
```shell
rosrun openimu_driver openimu_driver_node
```

## Choose GPSTIME(Default) or UNIXTIME
openrtk_node.cpp
```c=27
imu.header.stamp = ros::Time(sec ,nsec); //GPSTIME
// imu.header.stamp = ros::Time::now(); //UNIXTIME
```

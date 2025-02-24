# ROS下Xela-uSkin传感器启动与配置指南

## 一、环境准备
### 创建工作空间与功能包
```bash
mkdir -p xela_ws/src
cd xela_ws/src
catkin_create_pkg xela_driver rospy roscpp sensor_msgs
```

复制xela官网资料里的linux相关驱动代码,命名为 xela_linx/
```bash
ls xela_linx/
LOG  slcan_activate.sh  xela_conf  xela_conf.log  xela_error.log  xela_log  xela_server  xelatools.log  xela_viz  xela_viz.log  xServ.ini
```
和CAN相关驱动 VScom_USB-CAN_Plus/
```bash
ls VScom_USB-CAN_Plus/
99-VScom_USB-CAN_Plus.rules  install.sh  README.md  remove.sh  slcan_add.sh  slcan@.service```
```
相关ROS驱动为 xela_server_ros/
```bash
ls xela_server_ros/
CMakeLists.txt  launch  LICENSE  msg  package.xml  readme.md  scripts  srv
```
系统为ubuntu20.04


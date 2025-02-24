# ROS下Xela-uSkin传感器配置与启动指南

<!-- ## 一、环境准备 -->
### 创建工作空间与功能包
安装依赖(安装后才能使用slcan)
```bash
sudo apt install can-utils
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
系统为ubuntu20.04 (测试过22.04无法使用)

首先在VScom_USB-CAN_Plus/目录下执行install.sh脚本，安装CAN驱动，
然后在xela_linx/下执行slcan_activate.sh脚本，激活CAN设备。
执行成功后USB-CAN设备会亮绿灯
```bash
sudo ./VScom_USB-CAN_Plus/install.sh #一般第一次配置正常后，后续不需要每次再执行
sudo ./xela_linx/slcan_activate.sh 0
```
在xela_linx/下执行xela_server脚本，
启动xela_server服务，启动完不能关
```bash
sudo ./xela_linx/xela_server
```
另外开一个终端执行xela_linx/中xela_viz文件
，如正常则会显示三维力可视化界面
```bash
sudo ./xela_linx/xela_viz
```




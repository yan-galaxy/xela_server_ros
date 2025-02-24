# ROS下Xela-uSkin传感器配置与启动指南

<!-- ## 一、环境准备 -->
<!-- ### 创建工作空间与功能包 -->
### 首先检查接线 先连黑色USB供电线，然后连接can的USB
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
## 系统为ubuntu20.04 (测试过22.04无法使用)

安装依赖(安装后才能使用slcan)
```bash
sudo apt install can-utils
```
创建/etc/xela目录 将xServ.ini复制进去
```bash
cd /etc
mkdir xela/
cp /home/galaxy/Desktop/Xela_ws/src/xela_linx/xServ.ini #修改为自己的路径
```
实际使用时xServ.ini内容如下
```bash
[CAN]
bustype = socketcan
channel = slcan0
[viz]
max_offset = 100
max_size = 100
grid = on
origins = off
arrows = off
transparency = off
[debug]
sens_print = full
[sensor]
calibration = on # 开了校准之后ROS节点才会有换算成N的数据消息
num_brd = 1
ctr_ver = 3
ctrl_id = 4
model = uSPa44
channel = 0

```
先查看USB设备名
在xela_linx/下执行slcan_activate.sh脚本 激活CAN设备

执行成功后USB-CAN设备会亮绿灯

或者配置为自动启动 在VScom_USB-CAN_Plus/目录下执行install.sh脚本 安装CAN驱动
```bash
ls /dev/ttyUSB*
/dev/ttyUSB0
sudo ./xela_linx/slcan_activate.sh 0  # 0就是USB端口号ttyUSB0
或
sudo ./VScom_USB-CAN_Plus/install.sh #一般第一次配置正常后，后续不需要每次再执行
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
## ROS下消息传输



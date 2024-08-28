cmake_minimum_required(VERSION 3.0.2)
project(threed_viz)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  visualization_msgs  # 添加对visualization_msgs的依赖
  tf                  # 添加对tf的依赖
)
add_message_files(
  FILES
  "stm32data.msg"
  "robotiq3fctrl.msg"
  "robotiq3f_feedback.msg"
)
# add_service_files(
#   FILES
#   "XelaSensorStream.srv"
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES threed_viz
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
)
#添加pthread
set(CMAKE_CXX_FLAGS -pthread)
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

# 添加可执行目标
add_executable(subscriber_sensor src/subscriber_sensor.cpp)

# 链接ROS包
target_link_libraries(subscriber_sensor ${catkin_LIBRARIES} )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIRS})



file(
GLOB SRC_LIST 
${CMAKE_CURRENT_SOURCE_DIR}/src/serial_robotiq3f.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/modbus_crc16.cpp
)

add_executable(serial_robotiq3f ${SRC_LIST})
# 链接ROS包
target_link_libraries(serial_robotiq3f ${catkin_LIBRARIES} )
add_dependencies(serial_robotiq3f ${PROJECT_NAME}_gencpp)# 链接自定义消息必须添加的

file(
GLOB MOTION_SRC_LIST 
${CMAKE_CURRENT_SOURCE_DIR}/src/draw_chart.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/motion_control.cpp
)
add_executable(motion_control ${MOTION_SRC_LIST})
target_link_libraries(motion_control ${catkin_LIBRARIES})# 主流程运动控制
add_dependencies(motion_control ${PROJECT_NAME}_gencpp)# 链接自定义消息必须添加的

target_link_libraries(motion_control ${OpenCV_LIBS})
#############
## Install ##
#############



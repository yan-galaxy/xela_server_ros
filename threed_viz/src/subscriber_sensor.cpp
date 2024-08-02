#include "ros/ros.h"
#include "xela_server_ros/SensStream.h"
#include <visualization_msgs/Marker.h>
#include <thread>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"  // 包含Header消息的头文件
#include "std_msgs/ColorRGBA.h"  // 包含ColorRGBA消息的头文件

#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <cmath> // 用于 M_PI, atan2 等函数

typedef struct
{
    double x;
    double y;
    double z;
    double len;
}SEN_COO;
SEN_COO sen_coo[16];
SEN_COO sen_all;
tf2::Quaternion xyz2quaternion(double x,double y,double z)
{
    // 计算矢量的长度（模）
    double length = std::sqrt(x * x + y * y + z * z);

    // 确保是单位向量，如果不是，进行归一化
    x /= length;
    y /= length;
    z /= length;

    double pitch = atan2(-z, std::hypot(x, y));//-z  向下为正 向上为负
    double roll = 0;
    double yaw = atan2(y, x);

    // 将弧度转换为角度
    double roll_deg = roll * (180.0 / M_PI);
    double pitch_deg = pitch * (180.0 / M_PI);
    double yaw_deg = yaw * (180.0 / M_PI);

    // 使用tf2::Quaternion创建四元数
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    // // 输出结果
    // std::cout << "Roll (deg): " << roll_deg << std::endl;
    // std::cout << "Pitch (deg): " << pitch_deg << std::endl;
    // std::cout << "Yaw (deg): " << yaw_deg << std::endl;
    // std::cout << "Quaternion: (" 
    //           << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")" << std::endl;
    
    return q;
}


// 定义回调函数，它将在接收到消息时被调用
void callback(const xela_server_ros::SensStream::ConstPtr& msg)
{
    // 遍历所有传感器数据
    for (const auto& sensor : msg->sensors)
    {
        // ROS_INFO("Received data from sensor with message ID: %d", sensor.message);
        // ROS_INFO("Timestamp: %f", sensor.time);
        // ROS_INFO("Model: %s", sensor.model.c_str());
        // ROS_INFO("Sensor Position: %d", sensor.sensor_pos);

        // // 打印触觉像素数据
        // ROS_INFO("Taxels:");
        // for (const auto& taxel : sensor.taxels)
        // {
        //     ROS_INFO("X: %d, Y: %d, Z: %d", taxel.x, taxel.y, taxel.z);
        // }

        // // 打印力数据
        // ROS_INFO("Forces:");
        // for (const auto& force : sensor.forces)
        // {
        //     ROS_INFO("X: %f, Y: %f, Z: %f", force.x, force.y, force.z);
        // }
        
        SEN_COO sen_temp;
        sen_temp.x=0;
        sen_temp.y=0;
        sen_temp.z=0;
        for(uint8_t i=0;i<16;i++)
        {
            sen_coo[i].x=msg->sensors[0].forces[i].x;
            sen_coo[i].y=msg->sensors[0].forces[i].y;
            sen_coo[i].z=msg->sensors[0].forces[i].z;
            sen_temp.x+=sen_coo[i].x;
            sen_temp.y+=sen_coo[i].y;
            sen_temp.z+=sen_coo[i].z;
            // ROS_INFO("X: %f, Y: %f, Z: %f", msg->sensors[0].forces[i].x, msg->sensors[0].forces[i].y, msg->sensors[0].forces[i].z);
        }
        sen_all.x=sen_temp.x;
        sen_all.y=sen_temp.y;
        sen_all.z=sen_temp.z;
        sen_all.len=std::sqrt(sen_all.x * sen_all.x + sen_all.y * sen_all.y + sen_all.z * sen_all.z);
        ROS_INFO("ALL X: %f, Y: %f, Z: %f,len: %f",sen_all.x ,sen_all.y ,sen_all.z,sen_all.len);
    }
}

void viz_maker_proj(ros::Publisher pub,ros::Rate rosrate)
{
    uint32_t shape = visualization_msgs::Marker::ARROW;// CUBE SPHERE ARROW CYLINDER LINE_STRIP LINE_LIST CUBE_LIST SPHERE_LIST POINTS
    
    double x = 2.0; // x分量
    double y = 1.0; // y分量
    double z = 1.0; // z分量

    tf2::Quaternion q;
    // q=xyz2quaternion(x,y,z);//x y z

    visualization_msgs::Marker marker[19];
    for(size_t i=0;i<19;i++)
    {
        marker[i].header.frame_id = "sensor"; // 指定坐标系
        marker[i].header.stamp = ros::Time::now();
        marker[i].ns = "arrow"; // 命名空间
        marker[i].id = i; // 唯一标识符
        marker[i].type = visualization_msgs::Marker::ARROW; // 箭头类型
        marker[i].action = visualization_msgs::Marker::ADD; // 添加标记  // ADD MODIFY DELETE DELETEALL
        // 设置箭头的生命周期
        marker[i].lifetime = ros::Duration(); // 永久存在
    }

    for(uint8_t k=0;k<4;k++)
    {
        for(uint8_t j=0;j<4;j++)
        {
            marker[k*4+j].pose.position.x = (double)j*5;//起点位置
            marker[k*4+j].pose.position.y = (double)k*5;
            marker[k*4+j].pose.position.z = 2.0;

            // 设置箭头的颜色
            marker[k*4+j].color.r = 0.0; // 红色
            marker[k*4+j].color.g = 1.0; // 绿色
            marker[k*4+j].color.b = 0.0; // 蓝色
            marker[k*4+j].color.a = 1.0; // 透明度
        }
    }

    // // 设置箭头的位置
    // marker[0].pose.position.x = 0.0;//起点位置
    // marker[0].pose.position.y = 0.0;
    // marker[0].pose.position.z = 0.0;

    // marker[0].pose.orientation.x = q.x();//箭头朝向
    // marker[0].pose.orientation.y = q.y();
    // marker[0].pose.orientation.z = q.z();//
    // marker[0].pose.orientation.w = q.w();//实部 其他都为0时箭头指向x轴正方向

    // // 设置箭头的尺寸
    // marker[0].scale.x = 4.0; // 箭头的长度
    // marker[0].scale.y = 0.1; // 箭头的截面宽度1
    // marker[0].scale.z = 0.1; // 箭头的截面宽度2

    // // 设置箭头的颜色
    // marker[0].color.r = 1.0; // 红色
    // marker[0].color.g = 0.0; // 绿色
    // marker[0].color.b = 1.0; // 蓝色
    // marker[0].color.a = 1.0; // 透明度

    ROS_INFO("123");
    while(ros::ok())
    {
        for(size_t i=0;i<16;i++)
        {
            q=xyz2quaternion(sen_coo[i].x,sen_coo[i].y,sen_coo[i].z);//x y z

            marker[i].pose.orientation.x = q.x();
            marker[i].pose.orientation.y = q.y();
            marker[i].pose.orientation.z = q.z();
            marker[i].pose.orientation.w = q.w();

            marker[i].scale.x = std::sqrt(sen_coo[i].x * sen_coo[i].x + sen_coo[i].y * sen_coo[i].y + sen_coo[i].z * sen_coo[i].z); // 箭头的长度
            marker[i].scale.y = std::sqrt(sen_coo[i].x * sen_coo[i].x + sen_coo[i].y * sen_coo[i].y + sen_coo[i].z * sen_coo[i].z)/20.0; // 箭头的截面宽度1
            marker[i].scale.z = std::sqrt(sen_coo[i].x * sen_coo[i].x + sen_coo[i].y * sen_coo[i].y + sen_coo[i].z * sen_coo[i].z)/20.0; // 箭头的截面宽度2
            
            // 发布Marker消息
            pub.publish(marker[i]);
        }
        // pub.publish(marker[0]);
        rosrate.sleep();
    }

}
int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "xela_sensor_subscriber");

    // 创建节点句柄
    ros::NodeHandle n1,n2;

    // 创建一个订阅者，订阅xServTopic话题，消息类型为SensStream，回调函数为callback
    ros::Subscriber sub = n1.subscribe("xServTopic", 1000, callback);

    ros::Publisher marker_pub = n2.advertise<visualization_msgs::Marker>("xela_marker", 50);
    ros::Rate loop_rate_pub(100);
    std::thread viz_maker_thread(viz_maker_proj,marker_pub,loop_rate_pub);//发布brainco控制信息 
    
    // viz_maker_thread.join();

    ros::spin();

    return 0;
}
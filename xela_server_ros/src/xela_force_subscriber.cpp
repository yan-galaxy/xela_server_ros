#include <ros/ros.h>
#include <xela_server_ros/SensStream.h>
#include <fstream>
#include <iostream>
#include <ctime>

// 全局变量用于保存文件流和是否是第一次写入
std::ofstream csv_file;
bool first_write = true;

void sensorCallback(const xela_server_ros::SensStream::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->sensors.size(); ++i) {
        const auto& sensor = msg->sensors[i];
        
        // 计算合力
        float total_force_x = 0.0f;
        float total_force_y = 0.0f;
        float total_force_z = 0.0f;
        // 打印所有forces数据
        // ROS_INFO_THROTTLE(0.1, "  Forces count: %lu", sensor.forces.size());
        for (size_t j = 0; j < sensor.forces.size(); ++j) {
            const auto& force = sensor.forces[j];
            // ROS_INFO("    Force[%lu] - X: %.3f, Y: %.3f, Z: %.3f", j, force.x, force.y, force.z);
            // 累加各方向力
            total_force_x += force.x;
            total_force_y += force.y;
            total_force_z += force.z;
        }
        // 打印合力
        ROS_INFO_THROTTLE(0.1,"    Total Force - X: % 8.3f, Y: % 8.3f, Z: % 8.3f", total_force_x, total_force_y, total_force_z);
        
        // 写入CSV文件
        if (csv_file.is_open()) {
            // 如果是第一次写入，先写入标题行
            if (first_write) {
                csv_file << "timestamp,sensor_id,total_force_x,total_force_y,total_force_z\n";
                first_write = false;
            }
            
            // 获取当前时间戳
            double timestamp = ros::Time::now().toSec();
            
            // 写入数据行
            csv_file << timestamp << "," << i << "," << total_force_x << "," << total_force_y << "," << total_force_z << "\n";
        }
    }
    
    // ROS_INFO("---");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xela_force_subscriber");
    ros::NodeHandle n;
    
    // 创建带时间戳的文件名
    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);
    char filename[100];
    std::sprintf(filename, "force_data_%04d%02d%02d_%02d%02d%02d.csv", 
                 now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
                 now->tm_hour, now->tm_min, now->tm_sec);
    
    // 打开CSV文件进行写入
    csv_file.open(filename);
    if (!csv_file.is_open()) {
        ROS_ERROR("Failed to open CSV file for writing: %s", filename);
        return -1;
    }
    
    ROS_INFO("Writing force data to CSV file: %s", filename);
    
    ros::Subscriber sub = n.subscribe("/xServTopic", 10, sensorCallback);
    
    ROS_INFO("XELA Sensor Subscriber started. Listening to /xServTopic");
    
    ros::spin();
    
    // 关闭文件
    if (csv_file.is_open()) {
        csv_file.close();
    }
    
    return 0;
}
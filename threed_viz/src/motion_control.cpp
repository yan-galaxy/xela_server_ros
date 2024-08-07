#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <thread>

#include <vector>
#include "draw_chart.h"
#include <ctime>
#include <cmath>

#include "threed_viz/robotiq3fctrl.h" 
#include "threed_viz/robotiq3f_feedback.h" 
#include "xela_server_ros/SensStream.h"
#include "sensor_brainco/stm32data.h"
threed_viz::robotiq3fctrl robotiq_Ctrl_msg;
threed_viz::robotiq3f_feedback robotiq3f_feedback_msg;
sensor_brainco::stm32data stm32data_msg;

std::vector<double> SensorX(std::vector<double>(0));
std::vector<double> SensorY(std::vector<double>(0));
std::vector<double> SensorZ(std::vector<double>(0));
volatile uint8_t ctrl_command;
typedef struct
{
    double x;
    double y;
    double z;
    double len;
}SEN_COO;
// 定义一阶巴特沃斯高通滤波器
class ButterworthHighPassFilter {
public:
    ButterworthHighPassFilter(double cutoffFreq, double sampleRate)
        : cutoffFreq(cutoffFreq), sampleRate(sampleRate), prevInput(0.0), prevOutput(0.0) {
        double RC = 1.0 / (cutoffFreq * 2 * M_PI);
        double dt = 1.0 / sampleRate;
        alpha = dt / (RC + dt);
    }

    double filter(double input) {
        double output = alpha * (prevOutput + input - prevInput);
        prevInput = input;
        prevOutput = output;
        return output;
    }

private:
    double cutoffFreq;
    double sampleRate;
    double alpha;
    double prevInput;
    double prevOutput;
};
SEN_COO sen_coo[16];
SEN_COO sen_all;
ButterworthHighPassFilter filter_x(5.0, 100.0);
ButterworthHighPassFilter filter_y(5.0, 100.0);
ButterworthHighPassFilter filter_z(5.0, 100.0);
double sen_filter_x;
double sen_filter_y;
double sen_filter_z;

ButterworthHighPassFilter filter_3(5.0, 100.0);
ButterworthHighPassFilter filter_5(5.0, 100.0);
ButterworthHighPassFilter filter_10(5.0, 100.0);
ButterworthHighPassFilter filter_11(5.0, 100.0);
double stm32filter_3;
double stm32filter_5;
double stm32filter_10;
double stm32filter_11;

std::string savepath = "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/";


void stm32sen_callback(const sensor_brainco::stm32data::ConstPtr& msg)
{
    memcpy(&stm32data_msg,msg.get(),128);
    // ROS_INFO("10:%4d,11:%4d,3:%4d,5:%4d",stm32data_msg.voltage[10],stm32data_msg.voltage[11],stm32data_msg.voltage[3],stm32data_msg.voltage[5]);
    stm32filter_3=filter_3.filter(stm32data_msg.voltage[3]);
    stm32filter_5=filter_5.filter(stm32data_msg.voltage[5]);
    stm32filter_10=filter_10.filter(stm32data_msg.voltage[10]);
    stm32filter_11=filter_11.filter(stm32data_msg.voltage[11]);
    
    saveDataToFile(savepath+"stm32raw3.txt", stm32data_msg.voltage[3]);
    saveDataToFile(savepath+"stm32raw5.txt", stm32data_msg.voltage[5]);
    saveDataToFile(savepath+"stm32raw10.txt", stm32data_msg.voltage[10]);
    saveDataToFile(savepath+"stm32raw11.txt", stm32data_msg.voltage[11]);

    saveDataToFile(savepath+"stm32fil3.txt", stm32filter_3);
    saveDataToFile(savepath+"stm32fil5.txt", stm32filter_5);
    saveDataToFile(savepath+"stm32fil10.txt", stm32filter_10);
    saveDataToFile(savepath+"stm32fil11.txt", stm32filter_11);

}
void xsen_callback(const xela_server_ros::SensStream::ConstPtr& msg)
{
    // 获取系统当前时间的Unix时间戳
    time_t now = time(NULL);

    struct tm *timeinfo = localtime(&now); // 将 time_t 转换为 tm 结构，本地时间

    char time_buffer[80]; // 创建一个字符数组作为缓冲区
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", timeinfo); // 使用特定格式填充缓冲区
    // std::cout << "Current time: " << time_buffer << std::endl; // 打印格式化的时间字符串
    std::string time_str = time_buffer;
    // std::string time_str = timeToString(ros::Time::now());

    // 遍历所有传感器数据
    for (const auto& sensor : msg->sensors)
    {
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
        sen_filter_x=filter_x.filter(sen_all.x);
        sen_filter_y=filter_y.filter(sen_all.y);
        sen_filter_z=filter_z.filter(sen_all.z);
        saveDataToFile(savepath+"xela_x.txt", sen_all.x);
        saveDataToFile(savepath+"xela_y.txt", sen_all.y);
        saveDataToFile(savepath+"xela_z.txt", sen_all.z);
        saveDataToFile(savepath+"filter_x.txt", sen_filter_x);
        saveDataToFile(savepath+"filter_y.txt", sen_filter_y);
        saveDataToFile(savepath+"filter_z.txt", sen_filter_z);
        sen_all.len=std::sqrt(sen_all.x * sen_all.x + sen_all.y * sen_all.y + sen_all.z * sen_all.z);
        // ROS_INFO("ALL X: %f, Y: %f, Z: %f,len: %f",sen_all.x ,sen_all.y ,sen_all.z,sen_all.len);
    }
}
void robotiq3f_fb_callback(const threed_viz::robotiq3f_feedback::ConstPtr& msg)
{
    robotiq3f_feedback_msg.A_position=msg->A_position;
    robotiq3f_feedback_msg.A_current =msg->A_current;
    robotiq3f_feedback_msg.B_position=msg->B_position;
    robotiq3f_feedback_msg.B_current =msg->B_current;
    robotiq3f_feedback_msg.C_position=msg->C_position;
    robotiq3f_feedback_msg.C_current =msg->C_current;
    
}
void robotiq_Ctrl_pub(ros::Publisher pub,ros::Rate rosrate)//向brainco请求反馈数据,然后发布反馈数据fb
{
    robotiq_Ctrl_msg.position=0;
    robotiq_Ctrl_msg.speed=20;
    robotiq_Ctrl_msg.force=0;
    
    while(ros::ok())
    {
        // ROS_INFO("send");
        // 发布消息
        pub.publish(robotiq_Ctrl_msg);
        robotiq_Ctrl_msg.command=0;
        robotiq_Ctrl_msg.stop=0;

        // 按照循环频率延时
        rosrate.sleep();
    }
}
void robotiq_Ctrl_Once(uint8_t pos,uint8_t spe,uint8_t cur)
{
    if(pos>105)pos=105;
    robotiq_Ctrl_msg.position=pos;
    robotiq_Ctrl_msg.speed=spe;
    robotiq_Ctrl_msg.force=cur;
    robotiq_Ctrl_msg.command=1;
}
void robotiq_stop()
{
    robotiq_Ctrl_msg.command=1;
    robotiq_Ctrl_msg.stop=1;
}
void main_proj(ros::Publisher pub)//向brainco请求反馈数据,然后发布反馈数据fb
{
    uint8_t run_stat=0;
    sleep(1);
    robotiq_Ctrl_Once(0,20,0);
    sleep(3);
    robotiq_Ctrl_Once(105,10,0);//合拢
    ROS_INFO("motion");
    while(ros::ok())
    { 
        switch(run_stat)
        {
            case 0:
                if(sen_filter_z>0.1 && ( (stm32data_msg.diff[3]<-30)||(stm32data_msg.diff[5]<-30)   
                ||(stm32data_msg.diff[10]<-30)||(stm32data_msg.diff[11]<-30) ) )//三个手指都接触
                {
                    ROS_INFO("touch");
                    // robotiq_stop();
                    robotiq_Ctrl_Once(robotiq3f_feedback_msg.A_position+6,50,0);//停止闭合
                    run_stat=1;
                }
                break;

            case 1:
                sleep(15);
                ROS_INFO("down");
                run_stat=2;
                break;

            case 2:
                sleep(10);
                robotiq_Ctrl_Once(0,20,0);
                ROS_INFO("release");
                run_stat=3;
                break;

            case 3:
                break;

            case 4:
                break;

            case 5:
                break;



            case 10:
                break;

            

            case 20:
                break;


            default:
                break;
                
        }
        usleep(100*1);
    }

}
int main( int argc, char** argv )
{
    ros::init(argc, argv, "motion_control");

    ros::NodeHandle n1,n2,n3;
    clearFileContent(savepath+"xela_x.txt");
    clearFileContent(savepath+"xela_y.txt");
    clearFileContent(savepath+"xela_z.txt");
    clearFileContent(savepath+"filter_x.txt");
    clearFileContent(savepath+"filter_y.txt");
    clearFileContent(savepath+"filter_z.txt");

    clearFileContent(savepath+"stm32raw3.txt");
    clearFileContent(savepath+"stm32raw5.txt");
    clearFileContent(savepath+"stm32raw10.txt");
    clearFileContent(savepath+"stm32raw11.txt");
    clearFileContent(savepath+"stm32fil3.txt");
    clearFileContent(savepath+"stm32fil5.txt");
    clearFileContent(savepath+"stm32fil10.txt");
    clearFileContent(savepath+"stm32fil11.txt");

    ros::Subscriber sub1 = n1.subscribe("xServTopic", 1000, xsen_callback);
    ros::Subscriber sub2 = n2.subscribe("stm32data_info", 1000, stm32sen_callback);
    ros::Subscriber sub3 = n3.subscribe("robotiq3f_feedback", 1000, robotiq3f_fb_callback);
    ros::Publisher robotiq_Ctrl_info_pub = n3.advertise<threed_viz::robotiq3fctrl>("/robotiq3fctrl", 10);
    ros::Rate loop_rate_pub(200);
    
    std::thread ros_robotiq_Ctrl_pub_thread(robotiq_Ctrl_pub,robotiq_Ctrl_info_pub,loop_rate_pub); 
    
    std::thread main_proj_thread(main_proj,robotiq_Ctrl_info_pub);
    
    ros::spin();
}
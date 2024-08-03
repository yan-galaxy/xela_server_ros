#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <thread>
#include "threed_viz/robotiq3fctrl.h" 
#include "xela_server_ros/SensStream.h"
#include "sensor_brainco/stm32data.h"
threed_viz::robotiq3fctrl robotiq_Ctrl_msg;
sensor_brainco::stm32data stm32data_msg;
volatile uint8_t ctrl_command;
typedef struct
{
    double x;
    double y;
    double z;
    double len;
}SEN_COO;
SEN_COO sen_coo[16];
SEN_COO sen_all;
void stm32sen_callback(const sensor_brainco::stm32data::ConstPtr& msg)
{
    memcpy(&stm32data_msg,msg.get(),128);
    // ROS_INFO("10:%4d,11:%4d,3:%4d,5:%4d",stm32data_msg.voltage[10],stm32data_msg.voltage[11],stm32data_msg.voltage[3],stm32data_msg.voltage[5]);
}
void xsen_callback(const xela_server_ros::SensStream::ConstPtr& msg)
{
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
        sen_all.len=std::sqrt(sen_all.x * sen_all.x + sen_all.y * sen_all.y + sen_all.z * sen_all.z);
        // ROS_INFO("ALL X: %f, Y: %f, Z: %f,len: %f",sen_all.x ,sen_all.y ,sen_all.z,sen_all.len);
    }
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

        // 按照循环频率延时
        rosrate.sleep();
    }
}
void robotiq_Ctrl_Once(uint8_t pos,uint8_t spe,uint8_t cur)
{
    robotiq_Ctrl_msg.position=pos;
    robotiq_Ctrl_msg.speed=spe;
    robotiq_Ctrl_msg.force=cur;
    robotiq_Ctrl_msg.command=1;
}
void main_proj(ros::Publisher pub)//向brainco请求反馈数据,然后发布反馈数据fb
{
    uint8_t run_stat=0;
    sleep(1);
    robotiq_Ctrl_Once(70,20,0);
    sleep(2);
    robotiq_Ctrl_Once(0,20,0);
    while(ros::ok())
    { 
        switch(run_stat)
        {
            case 0:
                if(robotiq_Ctrl_msg.position<105)
                {
                    // robotiq_Ctrl_msg.position++;
                    // robotiq_Ctrl_msg.speed=0;
                    // robotiq_Ctrl_msg.force=0;
                    usleep(1000*10);
                }
                if(sen_all.z>0.4 && ( (stm32data_msg.diff[3]<-50)||(stm32data_msg.diff[5]<-50) )  
                && ( (stm32data_msg.diff[10]<-50)||(stm32data_msg.diff[11]<-50) ) )//三个手指都接触
                {
                    ROS_INFO("touch");
                    run_stat=1;
                }
                break;

            case 1:
                break;

            case 2:
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

    ros::Subscriber sub1 = n1.subscribe("xServTopic", 1000, xsen_callback);
    ros::Subscriber sub2 = n2.subscribe("stm32data_info", 1000, stm32sen_callback);
    ros::Publisher robotiq_Ctrl_info_pub = n3.advertise<threed_viz::robotiq3fctrl>("/robotiq3fctrl", 10);
    ros::Rate loop_rate_pub(200);
    
    std::thread ros_robotiq_Ctrl_pub_thread(robotiq_Ctrl_pub,robotiq_Ctrl_info_pub,loop_rate_pub); 
    
    std::thread main_proj_thread(main_proj,robotiq_Ctrl_info_pub);

    ros::spin();
}
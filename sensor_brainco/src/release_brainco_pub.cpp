#include <ros/ros.h>
#include "sensor_brainco/brainco_.h"

sensor_brainco::brainco_ brainco_ctrl_msg;

int main(int argc, char **argv) 
{

    // ROS节点初始化
    ros::init(argc, argv, "release_brainco_pub");

    // 创建节点句柄
    ros::NodeHandle n;

    ros::Publisher brainco_Ctrl_info_pub = n.advertise<sensor_brainco::brainco_>("/brainco/brainco_Ctrl_info", 38);//发布brainco ctrl话题控制brainco
    ros::Rate loop_rate_pub(1000);

    brainco_ctrl_msg.which_choose=sensor_brainco::brainco_::choose_pos;//choose_pos choose_speed choose_current
    brainco_ctrl_msg.pos_ctrl[0]=0;
    brainco_ctrl_msg.pos_ctrl[1]=100;
    brainco_ctrl_msg.pos_ctrl[2]=0;
    brainco_ctrl_msg.pos_ctrl[3]=0;
    brainco_ctrl_msg.pos_ctrl[4]=0;
    brainco_ctrl_msg.pos_ctrl[5]=0;

    brainco_ctrl_msg.speed_ctrl[0]=-10;
    brainco_ctrl_msg.speed_ctrl[1]=0;
    brainco_ctrl_msg.speed_ctrl[2]=-20;
    brainco_ctrl_msg.speed_ctrl[3]=-20;
    brainco_ctrl_msg.speed_ctrl[4]=-20;
    brainco_ctrl_msg.speed_ctrl[5]=-20;

    brainco_ctrl_msg.current_ctrl[0]=0;
    brainco_ctrl_msg.current_ctrl[1]=0;
    brainco_ctrl_msg.current_ctrl[2]=-20;
    brainco_ctrl_msg.current_ctrl[3]=-20;
    brainco_ctrl_msg.current_ctrl[4]=-20;
    brainco_ctrl_msg.current_ctrl[5]=-20;

    brainco_ctrl_msg.shutdown=1;
    usleep(1000*500);
    // while(ros::ok())
    {
        for(size_t i=0;i<100;i++)
        {
            // 发布消息
            brainco_Ctrl_info_pub.publish(brainco_ctrl_msg);
            ROS_INFO("pos_ctrl[0]:%d,pos_ctrl[1]:%d",brainco_ctrl_msg.pos_ctrl[0],brainco_ctrl_msg.pos_ctrl[1]);

            loop_rate_pub.sleep();
        }
        usleep(1000*500);
    }


    return 0;

}
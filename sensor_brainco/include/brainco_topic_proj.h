#ifndef _BRAINCO_TOPIC_PROJ_H
#define _BRAINCO_TOPIC_PROJ_H



#include <ros/ros.h>
#include "softCRC.h"//自引
#include "serial_self.h"//自引
#include <stdio.h>
#include <thread>
#include <queue>
#include <mutex>
#include "sensor_brainco/brainco_.h"//msg
#include <atomic>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>





int send_pos_ctrl(int serial_fd,POS_union* pos_u,
uint8_t pos0,uint8_t pos1,uint8_t pos2,uint8_t pos3,uint8_t pos4,uint8_t pos5);
int send_pos_ctrl_direct(int serial_fd,POS_union* pos_u);

int send_speed_ctrl(int serial_fd,SPEED_union* speed_u,
uint8_t speed0,uint8_t speed1,uint8_t speed2,uint8_t speed3,uint8_t speed4,uint8_t speed5);
int send_speed_ctrl_direct(int serial_fd,SPEED_union* speed_u);

int send_current_ctrl(int serial_fd,CURRENT_union* current_u,
uint8_t current0,uint8_t current1,uint8_t current2,uint8_t current3,uint8_t current4,uint8_t current5);
int send_current_ctrl_direct(int serial_fd,CURRENT_union* current_u);

int data_request(int serial_fd,int8_t* pos,int8_t* speed,int8_t* current);

void signalHandler(int signal) ;

void brainco_ctrl_InfoCallback(const sensor_brainco::brainco_::ConstPtr& msg);
void brainco_fb_pub(ros::Publisher pub,ros::Rate rosrate);

void jntPubTimerCallback(const ros::TimerEvent&);




#endif
#ifndef _DATA_SENSOR_VISUAL_H
#define _DATA_SENSOR_VISUAL_H

#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <unistd.h> 
#include <vector>
#include "sensor_brainco/stm32data.h" //sensor_brainco  uart_stm32
#include <mutex>
#include <stdlib.h>
#include "WzSerialportPlus.h"
#include <queue>
#include <atomic>
#include <signal.h>
#include <thread>

uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen );


void set_para(
visualization_msgs::Marker * sel_marker,
float posx,float posy,float posz,
float orix,float oriy,float oriz,float oriw,
float scax,float scay,float scaz,
float colorR,float colorG,float colorB,float colorAlpha);

void GrayToPseColor(uint16_t grayValue, float *colorR,float *colorG,float *colorB);
void GrayToPseColor_rainbow1(uint16_t grayValue, float *colorR,float *colorG,float *colorB);

//信号处理函数，当接收到SIGINT信号（由Ctrl+C产生）时，它会设置keepRunning为false。
void signalHandler(int signal);



void visual_proj(ros::Publisher pub,ros::Rate rosrate);
void serial_callback(char* data,int length);
void serial_proj(WzSerialportPlus WzSerialport);
void data_pro_proj();
void ros_to_sensor_proj(ros::Publisher pub,ros::Rate rosrate);
#endif
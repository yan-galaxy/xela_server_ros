#ifndef _HEAD_H
#define _HEAD_H

#include <iostream>
#include <visualization_msgs/Marker.h>
#include <unistd.h> 
#include <fstream>
#include <vector>
#include <cmath>
#include "sensor_brainco/stm32data.h" //sensor_brainco  uart_stm32
#include "sensor_brainco/brainco.h"
#include <mutex>
#include <stdlib.h>
#include "WzSerialportPlus.h"
#include <queue>
#include <atomic>
#include <signal.h>
#include <ros/ros.h>
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

#endif
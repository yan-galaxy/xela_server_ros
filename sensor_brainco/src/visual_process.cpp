#include <visualization_msgs/Marker.h>
// #include "uart_stm32/stm32data.h"

void set_para(
visualization_msgs::Marker * sel_marker,
float posx,float posy,float posz,
float orix,float oriy,float oriz,float oriw,
float scax,float scay,float scaz,
float colorR,float colorG,float colorB,float colorAlpha)
{
    sel_marker->pose.position.x = posx;//0;
    sel_marker->pose.position.y = posy;//0;
    sel_marker->pose.position.z = posz;//0.25;

    sel_marker->pose.orientation.x = orix;//0.3;
    sel_marker->pose.orientation.y = oriy;//0.0;
    sel_marker->pose.orientation.z = oriz;//0.0;
    sel_marker->pose.orientation.w = oriw;//1;

    sel_marker->scale.x = scax;//2.0;
    sel_marker->scale.y = scay;//2.0;
    sel_marker->scale.z = scaz;//0.5;

    sel_marker->color.r = colorR;//0.0f;
    sel_marker->color.g = colorG;//1.0f;
    sel_marker->color.b = colorB;//0.0f;
    sel_marker->color.a = colorAlpha;//1.0;
}
void GrayToPseColor(uint16_t grayValue, float *colorR,float *colorG,float *colorB)
{
    // *colorR=(float)abs(0-grayValue)/256.0;
    // *colorG=(float)abs(127-grayValue)/256.0;
    // *colorB=(float)abs(255-grayValue)/256.0;

    if( (grayValue>=0) && (grayValue<=1023) )  
    {
        *colorR=0;
        *colorG=0;
        *colorB=(float)grayValue/1024.0;
    }
    else if( (grayValue>=1024) && (grayValue<=2047) )  
    {
        *colorR=0;
        *colorG=(float)(grayValue-1024)/1024.0;
        *colorB=(float)(2047-grayValue)/1024.0;
    }
    else if( (grayValue>=2048) && (grayValue<=3071) )  
    {
        *colorR=(float)(grayValue-2048)/1024.0;
        *colorG=1.0;
        *colorB=0;
    }
    else if( (grayValue>=3072) && (grayValue<=4095) )  
    {
        *colorR=1.0;
        *colorG=(float)(4095-grayValue)/1024.0;
        *colorB=0;
    }
}

void GrayToPseColor_rainbow1(uint16_t grayValue, float *colorR,float *colorG,float *colorB)
{
    // *colorR=(float)abs(0-grayValue)/256.0;
    // *colorG=(float)abs(127-grayValue)/256.0;
    // *colorB=(float)abs(255-grayValue)/256.0;

    if( (grayValue>=0) && (grayValue<=511) )  
    {
        *colorR=0;
        *colorG=0;
        *colorB=(float)grayValue/512.0;
    }
    else if( (grayValue>=512) && (grayValue<=1023) )  
    {
        *colorR=0;
        *colorG=(float)(grayValue-512)/512.0;
        *colorB=1.0;
    }
    else if( (grayValue>=1024) && (grayValue<=1535) )  
    {
        *colorR=0;
        *colorG=1.0;
        *colorB=(float)(1535-grayValue)/512.0;
    }
    else if( (grayValue>=1536) && (grayValue<=2047) )  
    {
        *colorR=(float)(grayValue-1536)/512.0;
        *colorG=1.0;
        *colorB=0;
    }
    else if( (grayValue>=2048) && (grayValue<=3071) )  
    {
        *colorR=1.0;
        *colorG=(float)(3071-grayValue)/1024.0;
        *colorB=0;
    }
    else if( (grayValue>=3072) && (grayValue<=4095) )  
    {
        *colorR=1.0;
        *colorG=(float)(grayValue-3072)/1024.0;
        *colorB=(float)(grayValue-3072)/1024.0;
    }
}
/* specifically for storing char data in embedded system */

#ifndef _SERIAL_SELF_H
#define _SERIAL_SELF_H

#include <stdint.h>
#include <stddef.h>


#include <termios.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> //文件控制定义
#include <termios.h>//终端控制定义
#include <errno.h>
#include <unistd.h>
#include <string.h>
//打开串口并初始化设置 

typedef union 
{
    uint8_t data[28];
    struct 
    {
        uint8_t head[4]={0x42,0x6E,0x43,0x50};
        uint8_t addr[2]={0x02,0x01};
        uint8_t num[2]={0x00,0x0E};
        uint8_t order[8]={0x12,0x0C,0x08,0x01,0x12,0x08,0x0A,0x06};
        int8_t value[6]={10,100,50,50,50,50};
        uint8_t nc[2]={0x00,0x00};
        uint8_t crc8[4];
    };
    
}POS_union;

typedef union 
{
    uint8_t data[28];
    struct 
    {
        uint8_t head[4]={0x42,0x6E,0x43,0x50};
        uint8_t addr[2]={0x02,0x01};
        uint8_t num[2]={0x00,0x0E};
        uint8_t order[8]={0x12,0x0C,0x08,0x02,0x12,0x08,0x12,0x06};
        int8_t value[6]={0,0,0,0,0,0};
        uint8_t nc[2]={0x00,0x00};
        uint8_t crc8[4];
    };
    
}SPEED_union;

typedef union 
{
    uint8_t data[28];
    struct 
    {
        uint8_t head[4]={0x42,0x6E,0x43,0x50};
        uint8_t addr[2]={0x02,0x01};
        uint8_t num[2]={0x00,0x0E};
        uint8_t order[8]={0x12,0x0C,0x08,0x03,0x12,0x08,0x1A,0x06};
        int8_t value[6]={0,0,0,0,0,0};
        uint8_t nc[2]={0x00,0x00};
        uint8_t crc8[4];
    };
    
}CURRENT_union;

int init_serial(char * port,int baudrate );
int uart_send(int fd, char *data, int datalen);
int uart_recv(int fd, char *data, int datalen);



#endif /* _SOFTCRC_H */

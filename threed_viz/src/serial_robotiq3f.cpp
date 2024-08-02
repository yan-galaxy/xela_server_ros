#include "ros/ros.h"
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
#include "modbus_crc16.h"

#include <atomic>
#include <signal.h>
#include <thread>

#include "threed_viz/robotiq3fctrl.h" 
#include "threed_viz/robotiq3f_feedback.h" 
#define DEVICE "/dev/ttyACM0" 

#define S_TIMEOUT 1
  
int serial_fd = 0; 
  
//打开串口并初始化设置 

int init_serial_(const char * port,int baudrate ) 
{ 
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY); 
    if (serial_fd < 0) { 
        perror("open"); 
        return -1; 
    } 
      
    //串口主要设置结构体termios <termios.h> 
    struct termios options; 
      
    /**1. tcgetattr函数用于获取与终端相关的参数。 
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中 
    */ 
    tcgetattr(serial_fd, &options); 
    /**2. 修改所获得的参数*/ 
    options.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能 
    options.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位 
    options.c_cflag &= ~CRTSCTS;//无硬件流控 
    options.c_cflag |= CS8;//8位数据长度 
    options.c_cflag &= ~CSTOPB;//1位停止位 
    options.c_iflag |= IGNPAR;//无奇偶检验位 
    options.c_oflag = 0; //输出模式 
    
    // options.c_lflag = 0; //不激活终端模式 

    // 设置为非规范模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // 设置最小接收字符数为0
    options.c_cc[VMIN] = 0;
    // 设置超时时间，例如设置为1秒（10 * 十分之一秒）
    options.c_cc[VTIME] = 20;

    // cfsetospeed(&options, B921600);//设置波特率 
    // cfsetispeed(&options, B921600);//设置波特率 
    cfsetspeed(&options, baudrate);//设置波特率 
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/ 
    tcflush(serial_fd, TCIFLUSH);//溢出数据可以接收，但不读 
    tcsetattr(serial_fd, TCSANOW, &options); 
      
    return serial_fd; 
} 
  
/** 
*串口发送数据 
*@fd:串口描述符 
*@data:待发送数据 
*@datalen:数据长度 
*/ 
unsigned int total_send = 0 ;
int uart_send_(int fd, char *data, int datalen) 
{ 
    int len = 0; 
    len = write(fd, data, datalen);//实际写入的长度 
    if(len == datalen) { 
    total_send += len ;
        // printf("total_send is %d\n",total_send); 
        return len; 
    } else { 
        tcflush(fd, TCOFLUSH);//TCOFLUSH刷新写入的数据但不传送 
        return -1; 
    } 
    return 0; 
} 
  
/** 
*串口接收数据 
*要求启动后，在pc端发送ascii文件 
*/
unsigned int total_length = 0 ; 
int uart_recv_(int fd, char *data, int datalen) 
{ 
    int len=0, ret = 0; 
    fd_set fs_read; 
    struct timeval tv_timeout; 
      
    FD_ZERO(&fs_read); 
    FD_SET(fd, &fs_read); 

#ifdef S_TIMEOUT    
    tv_timeout.tv_sec = ( 10*20/115200+2); 
    tv_timeout.tv_usec = 0; 
    ret = select(fd+1, &fs_read, NULL, NULL, NULL);
#elif
    ret = select(fd+1, &fs_read, NULL, NULL, tv_timeout);
#endif

// printf("ret = %d\n", ret); 
    //如果返回0，代表在描述符状态改变前已超过timeout时间,错误返回-1 
     
    if (FD_ISSET(fd, &fs_read)) { 
        len = read(fd, data, datalen); 
    total_length += len ;
        // printf("total len = %d\n", total_length); 
        return len; 
    } else { 
        perror("select"); 
        return -1; 
    } 
      
    return 0; 
} 

//激活命令  Activation Request
unsigned char buf1[]={0x09,0x10,
0x03,0xE8,//寄存器地址
0x00,0x03,//3个寄存器
0x06,//6个字节
0x01,0x00,//写入0x03E8
0x00,0x00,//写入0x03E9
0x00,0x00,//写入0x03EA
0x72,0xE1};
//查询是否激活完成  Read Gripper status until the activation is completed
unsigned char buf2[]={0x09,0x03,0x07,0xD0,0x00,0x01,0x85,0xCF};
//Close the Gripper at full speed and full force
unsigned char buf3[15]={0x09 ,0x10 ,0x03 ,0xE8 ,0x00 ,0x03 ,0x06 ,0x09 ,0x00 ,0x00 ,
0xFF ,//position
0xFF ,//speed
0xFF ,//force
0x42 ,0x29};

//Read Gripper status until the grip is completed
unsigned char buf4[]={0x09 ,0x03 ,0x07 ,0xD0 ,0x00 ,0x08 ,0x45 ,0xC9};
//Open the Gripper at full speed and full force
unsigned char buf5[15]={0x09,0x10,
0x03,0xE8,//写入的起始地址
0x00,0x03,//写入的寄存器数量
0x06,//写入的字节数
0x09,0x00,//写入 0x03E9
0x00,0x00 ,//position
0xFF ,//speed
0xFF ,//force
0x72 ,0x19};

//control Gripper  position  speed  force
unsigned char bufselfc[15]={0x09 ,0x10 ,0x03 ,0xE8 ,0x00 ,0x03 ,0x06 ,0x09 ,0x00 ,0x00 ,
0 ,//position   105
10 ,//speed
0 ,//force
};
//查询寄存器
unsigned char buf_requet[8]={0x09,0x03,
0x07,0xD2,
0x00,0x04,
};

void uart_send_crc(int fd,uint8_t * pucFrame,uint8_t len)
{
    uint16_t crc;
    crc=usMBCRC16(pucFrame,len-2);
    pucFrame[len-2]=crc%256;
    pucFrame[len-1]=crc/256;
    uart_send_(fd, (char *)pucFrame, len); 
}

void robotiqInfoCallback(const threed_viz::robotiq3fctrl::ConstPtr& msg)
{
    uint8_t rx_len=0;
    ROS_INFO("robo_recv");
    bufselfc[10]=msg->position;
    bufselfc[11]=msg->speed;
    bufselfc[12]=msg->force;
    // uart_send_crc(serial_fd, bufselfc, sizeof(bufselfc)); 

    // usleep(1000*20);
    // rx_len=uart_recv_(serial_fd, (char *)bufrx, 8);//8
    // for(size_t i=0;i<rx_len;i++)
    // {
    //     printf("%02X ",bufrx[i]);
    // }
    // printf("\n");
    // memset(bufrx,0,sizeof(bufrx));
    // ROS_INFO("position:%d speed:%d force:%d",msg->position,msg->speed,msg->force);
}
std::atomic<bool> keepRunning(true);
//信号处理函数，当接收到SIGINT信号（由Ctrl+C产生）时，它会设置keepRunning为false。
void signalHandler(int signal) 
{
    // std::cout << std::endl << "main Caught signal: " << signal << std::endl;
    keepRunning = false;
    // exit(100);
}
unsigned char bufrx[512] ; 
void ctrl_pro_proj()
{
    int rx_len=0;
    while(keepRunning)//keepRunning
    {
        // uart_send_crc(serial_fd, bufselfc, sizeof(bufselfc)); 
        // usleep(1000*20);
        // char rx1[8]={0};
        // for(uint8_t i=0;i<8;i++)
        // {
        //     uart_recv_(serial_fd, (rx1+i), 1);//8
        // }
        // for(size_t i=0;i<8;i++)
        // {
        //     printf("%02X ",(uint8_t)rx1[i]);
        // }
        // printf("\n");


        // usleep(1000*20);



        // uart_send_crc(serial_fd, buf_requet, sizeof(buf_requet));

        // usleep(1000*10);
        // // rx_len=uart_recv_(serial_fd, (char *)bufrx, 20);
        // char rx2[21]={0};
        // for(uint8_t i=0;i<21;i++)
        // {
        //     uart_recv_(serial_fd, (rx2+i), 1);//8
        // }
        // for(size_t i=0;i<21;i++)
        // {
        //     printf("%02X ",(uint8_t)rx1[i]);
        // }
        // printf("\n");


        // memset(bufrx,0,sizeof(bufrx));

        usleep(1000*20);
    }
    ROS_INFO("over");
}
void ros_sub_proj()
{
    while(ros::ok() )//keepRunning
    {
        ros::spinOnce();//spinOnce spin
    }
}
int main(int argc, char **argv) 
{ 
    // 初始化节点
    ros::init(argc, argv, "serial_robotiq3f");
    init_serial_("/dev/ttyUSB1",B115200);

    ros::NodeHandle n1,n2;
    ros::Subscriber robotiq_info_sub = n1.subscribe("/robotiq3fctrl", 3, robotiqInfoCallback);//订阅传感器信息

    ros::Publisher robotiq_feedback_info_pub = n2.advertise<threed_viz::robotiq3f_feedback>("/robotiq3f_feedback", 10);
    ros::Rate loop_rate_pub(100);
    // std::thread ros_robotiq_Ctrl_pub_thread(robotiq_Ctrl_pub,robotiq_Ctrl_info_pub,loop_rate_pub); 

    // 设置信号处理函数
    signal(SIGINT, signalHandler);
    // ROS_INFO("recv");
    // ros::spin();
    std::thread ctrl_process_thread(ctrl_pro_proj);
    std::thread ros_sub_thread(ros_sub_proj);//接收控制信号

    
    ctrl_process_thread.join();
    ros_sub_thread.join();
    
    
    close(serial_fd); 
    return 0; 
}
#include "brainco_topic_proj.h"

std::mutex serial_mtx;//互斥锁
POS_union pos_u;
SPEED_union speed_u;
CURRENT_union current_u;//控制命令结构体
int serial_fd;

int8_t pos_read[6];
int8_t speed_read[6];
int8_t current_read[6];
std::atomic<bool> keepRunning(true);
sensor_brainco::brainco_ brainco_fb_msg;


sensor_msgs::JointState brainco_joint_state;
realtime_tools::RealtimePublisher<sensor_msgs::JointState> *ptr_jnt_pub;



int main(int argc, char **argv) 
{
    
    char port_str[]="/dev/ttyUSB1";
    serial_fd=init_serial(port_str,B115200);

    // ROS节点初始化
    ros::init(argc, argv, "brainco_topic_proj");

    // 创建节点句柄
    ros::NodeHandle n,n1;

    ros::Publisher brainco_fb_info_pub = n.advertise<sensor_brainco::brainco_>("/brainco/brainco_FB_info", 38);//发布brainco feedback
    ros::Rate loop_rate_pub(1000);

    // 创建一个Subscriber，订阅名为/brainco/brainco_ctrl_info的topic，注册回调函数brainco_ctrl_InfoCallback
    ros::Subscriber person_info_sub = n1.subscribe("/brainco/brainco_Ctrl_info", 38, brainco_ctrl_InfoCallback);//订阅外界的控制信息



    realtime_tools::RealtimePublisher<sensor_msgs::JointState>* joint_state_publisher = new 
        realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n,"/iiwa/brainco_joint_states",1);

    ptr_jnt_pub = joint_state_publisher;
    ros::Timer jntpub_timer = n.createTimer(ros::Duration(0.01),jntPubTimerCallback);



    std::thread ros_brainco_fb_pub_thread(brainco_fb_pub,brainco_fb_info_pub,loop_rate_pub);//发布brainco反馈的信息  
    
    // 设置信号处理函数
    // signal(SIGINT, signalHandler);//在这个程序里不好用

    ros::spin();

    close(serial_fd); 
}

void jntPubTimerCallback(const ros::TimerEvent&)
{
    if(ptr_jnt_pub->trylock()){
        ptr_jnt_pub->msg_=brainco_joint_state;
        ptr_jnt_pub->unlockAndPublish();
    }
}

//信号处理函数，当接收到SIGINT信号（由Ctrl+C产生）时，它会设置keepRunning为false。
void signalHandler(int signal) 
{
    std::cout << std::endl <<"brainCo Caught signal: " << signal << std::endl;
    keepRunning = false;
    // exit(100);
}

void brainco_fb_pub(ros::Publisher pub,ros::Rate rosrate)//向brainco请求反馈数据,然后发布反馈数据fb
{
    while(ros::ok() && keepRunning)
    {   
        data_request(serial_fd,pos_read,speed_read,current_read);//向brainco请求反馈数据
        
        memcpy(reinterpret_cast<int8_t*>(brainco_fb_msg.pos_fb.data())    ,pos_read    ,6);
        memcpy(reinterpret_cast<int8_t*>(brainco_fb_msg.speed_fb.data())  ,speed_read  ,6);
        memcpy(reinterpret_cast<int8_t*>(brainco_fb_msg.current_fb.data()),current_read,6);
        
        // data_request(serial_fd, reinterpret_cast<int8_t*>(brainco_fb_msg.pos_fb.data()),//向brainco请求反馈数据
        //      reinterpret_cast<int8_t*>(brainco_fb_msg.speed_fb.data()),
        //      reinterpret_cast<int8_t*>(brainco_fb_msg.current_fb.data()));

        // 发布消息
		pub.publish(brainco_fb_msg);
        
        if(brainco_fb_msg.which_choose==sensor_brainco::brainco_::choose_pos)
        {
            
            send_pos_ctrl_direct(serial_fd,&pos_u);
        }
        else if(brainco_fb_msg.which_choose==sensor_brainco::brainco_::choose_speed)
        {
            send_speed_ctrl_direct(serial_fd,&speed_u);
        }
        else if(brainco_fb_msg.which_choose==sensor_brainco::brainco_::choose_current)
        {
            send_current_ctrl_direct(serial_fd,&current_u);
        }
        brainco_fb_msg.which_choose=sensor_brainco::brainco_::choose_none;//发送后将状态位清除
        if(brainco_fb_msg.shutdown==1)ros::shutdown();

        brainco_joint_state.header.stamp = ros::Time::now();
        brainco_joint_state.name.clear();
        brainco_joint_state.position.clear();
        brainco_joint_state.velocity.clear();
        brainco_joint_state.effort.clear();
        for(uint8_t i=0;i<6;i++)
        {
            brainco_joint_state.name.push_back("finger_joint"+std::to_string(i+1));
            brainco_joint_state.position.push_back(pos_read[i]);
            brainco_joint_state.velocity.push_back(speed_read[i]);
            brainco_joint_state.effort.push_back(current_read[i]);
        }

        // 按照循环频率延时
        rosrate.sleep();

        
    }
}

// 接收到外界控制的订阅消息后，会进入消息回调函数，为防止与brainco控制命令和反馈信息冲突，把控制指令发出放在发布反馈数据fb之后进行发送
void brainco_ctrl_InfoCallback(const sensor_brainco::brainco_::ConstPtr& msg)
{
    static uint64_t cnt=0;
    cnt++;
    // 将接收到的消息打印出来
    // ROS_INFO("Subcribe cnt=%d",cnt);

    memcpy(&pos_u.value[0]    ,&msg->pos_ctrl[0]    ,6);
    memcpy(&speed_u.value[0]  ,&msg->speed_ctrl[0]  ,6);
    memcpy(&current_u.value[0],&msg->current_ctrl[0],6);

    brainco_fb_msg.which_choose=msg->which_choose;
    brainco_fb_msg.shutdown=msg->shutdown;
    // ROS_INFO("pos[0]:%d ,pos[1]:%d ,pos[2]:%d ,pos[3]:%d ,pos[4]:%d ,pos[5]:%d ",pos_u.value[0],pos_u.value[1],pos_u.value[2],pos_u.value[3],pos_u.value[4],pos_u.value[5]);
    
    // ROS_INFO("which:%d",req.which_choose);
    // if(msg->which_choose==sensor_brainco::brainco_::choose_pos)
    // {
    //     send_pos_ctrl(serial_fd,&pos_u,msg->pos_ctrl[0],msg->pos_ctrl[1],msg->pos_ctrl[2],msg->pos_ctrl[3],msg->pos_ctrl[4],msg->pos_ctrl[5]);
    // }
    // else if(msg->which_choose==sensor_brainco::brainco_::choose_speed)
    // {
    //     send_speed_ctrl(serial_fd,&speed_u,msg->speed_ctrl[0],msg->speed_ctrl[1],msg->speed_ctrl[2],msg->speed_ctrl[3],msg->speed_ctrl[4],msg->speed_ctrl[5]);
    // }
    // else if(msg->which_choose==sensor_brainco::brainco_::choose_current)
    // {
    //     send_current_ctrl(serial_fd,&current_u,msg->current_ctrl[0],msg->current_ctrl[1],msg->current_ctrl[2],msg->current_ctrl[3],msg->current_ctrl[4],msg->current_ctrl[5]);
    // }
}



int send_pos_ctrl(int serial_fd,POS_union* pos_u,
uint8_t pos0,uint8_t pos1,uint8_t pos2,uint8_t pos3,uint8_t pos4,uint8_t pos5)
{
    int tx_len=0;
    uint32_t crc32=0;

    pos_u->value[0]=pos0;
    pos_u->value[1]=pos1;
    pos_u->value[2]=pos2;
    pos_u->value[3]=pos3;
    pos_u->value[4]=pos4;
    pos_u->value[5]=pos5;

    crc32 = softCRC_CRC32(pos_u, (sizeof(*pos_u) - 4) / 4, 0xffffffff, 0);

    pos_u->crc8[0] = crc32 >> 24 & 0xFF;
    pos_u->crc8[1] = crc32 >> 16 & 0xFF;
    pos_u->crc8[2] = crc32 >> 8  & 0xFF;
    pos_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("tx_crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)pos_u, sizeof(*pos_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}
int send_pos_ctrl_direct(int serial_fd,POS_union* pos_u)
{
    int tx_len=0;
    uint32_t crc32=0;

    // pos_u->value[0]=pos0;
    // pos_u->value[1]=pos1;
    // pos_u->value[2]=pos2;
    // pos_u->value[3]=pos3;
    // pos_u->value[4]=pos4;
    // pos_u->value[5]=pos5;

    crc32 = softCRC_CRC32(pos_u, (sizeof(*pos_u) - 4) / 4, 0xffffffff, 0);

    pos_u->crc8[0] = crc32 >> 24 & 0xFF;
    pos_u->crc8[1] = crc32 >> 16 & 0xFF;
    pos_u->crc8[2] = crc32 >> 8  & 0xFF;
    pos_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("tx_crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)pos_u, sizeof(*pos_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}

int send_speed_ctrl(int serial_fd,SPEED_union* speed_u,
uint8_t speed0,uint8_t speed1,uint8_t speed2,uint8_t speed3,uint8_t speed4,uint8_t speed5)
{
    int tx_len=0;
    uint32_t crc32=0;

    speed_u->value[0]=speed0;
    speed_u->value[1]=speed1;
    speed_u->value[2]=speed2;
    speed_u->value[3]=speed3;
    speed_u->value[4]=speed4;
    speed_u->value[5]=speed5;

    crc32 = softCRC_CRC32(speed_u, (sizeof(*speed_u) - 4) / 4, 0xffffffff, 0);

    speed_u->crc8[0] = crc32 >> 24 & 0xFF;
    speed_u->crc8[1] = crc32 >> 16 & 0xFF;
    speed_u->crc8[2] = crc32 >> 8  & 0xFF;
    speed_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)speed_u, sizeof(*speed_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}
int send_speed_ctrl_direct(int serial_fd,SPEED_union* speed_u)
{
    int tx_len=0;
    uint32_t crc32=0;

    // speed_u->value[0]=speed0;
    // speed_u->value[1]=speed1;
    // speed_u->value[2]=speed2;
    // speed_u->value[3]=speed3;
    // speed_u->value[4]=speed4;
    // speed_u->value[5]=speed5;

    crc32 = softCRC_CRC32(speed_u, (sizeof(*speed_u) - 4) / 4, 0xffffffff, 0);

    speed_u->crc8[0] = crc32 >> 24 & 0xFF;
    speed_u->crc8[1] = crc32 >> 16 & 0xFF;
    speed_u->crc8[2] = crc32 >> 8  & 0xFF;
    speed_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)speed_u, sizeof(*speed_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}

int send_current_ctrl(int serial_fd,CURRENT_union* current_u,
uint8_t current0,uint8_t current1,uint8_t current2,uint8_t current3,uint8_t current4,uint8_t current5)
{
    int tx_len=0;
    uint32_t crc32=0;

    current_u->value[0]=current0;
    current_u->value[1]=current1;
    current_u->value[2]=current2;
    current_u->value[3]=current3;
    current_u->value[4]=current4;
    current_u->value[5]=current5;

    crc32 = softCRC_CRC32(current_u, (sizeof(*current_u) - 4) / 4, 0xffffffff, 0);

    current_u->crc8[0] = crc32 >> 24 & 0xFF;
    current_u->crc8[1] = crc32 >> 16 & 0xFF;
    current_u->crc8[2] = crc32 >> 8  & 0xFF;
    current_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)current_u, sizeof(*current_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}
int send_current_ctrl_direct(int serial_fd,CURRENT_union* current_u)
{
    int tx_len=0;
    uint32_t crc32=0;

    // current_u->value[0]=current0;
    // current_u->value[1]=current1;
    // current_u->value[2]=current2;
    // current_u->value[3]=current3;
    // current_u->value[4]=current4;
    // current_u->value[5]=current5;

    crc32 = softCRC_CRC32(current_u, (sizeof(*current_u) - 4) / 4, 0xffffffff, 0);

    current_u->crc8[0] = crc32 >> 24 & 0xFF;
    current_u->crc8[1] = crc32 >> 16 & 0xFF;
    current_u->crc8[2] = crc32 >> 8  & 0xFF;
    current_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)current_u, sizeof(*current_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}

int data_request(int serial_fd,int8_t* pos,int8_t* speed,int8_t* current)
{
    static uint32_t wrong_cnt=0,right_cnt=0;
    int tx_len=0;
    uint8_t rx_buf[512]={0}; 
    int rx_len=0;
    uint8_t request[16]={0x42,0x6E,0x43,0x50,0x02,0x01,0x00,0x04,0x4A,0x02,0x08,0x01,0xF2,0x7B,0xBD,0xE5};
    uint32_t calcu_crc32=0,recv_crc32=0;

    
    serial_mtx.lock();
    
    tx_len=uart_send(serial_fd, (char *)request, sizeof(request));//send data request
    usleep(sizeof(request)*300);
    usleep(88*160);
    memset(rx_buf,0,sizeof(rx_buf)); 
    rx_len=uart_recv(serial_fd, (char*)rx_buf, sizeof(rx_buf));
    serial_mtx.unlock();
    // printf("1234\n");
    

    calcu_crc32 = softCRC_CRC32(rx_buf, (rx_len - 4) / 4, 0xffffffff, 0);
    // printf("calcu_crc32:%08X\n",calcu_crc32);
    recv_crc32 = (rx_buf[rx_len - 4] << 24) + (rx_buf[rx_len - 3] << 16) + (rx_buf[rx_len - 2] << 8) + rx_buf[rx_len - 1];
    // printf("recv_crc32:%08X\n",recv_crc32);

    if(calcu_crc32!=recv_crc32 && rx_len==88)
    {
        wrong_cnt++;
        // printf("\nCRC wrong! %d %d\n\n",right_cnt,wrong_cnt);
        std::cerr << "CRC wrong! " << right_cnt <<" "<<wrong_cnt << std::endl;
        // perror("CRC wrong! \n");
        return -1;
    }
    if(rx_len==88)
    {
        right_cnt++;
    }

    // for(int i=0;i<rx_len;i++)
    // {
    //     // Queue.push(rx_buf[i]);
    //     if(i<(rx_len-1))
    //         printf("0x%02X,",rx_buf[i]);
    //     else 
    //         printf("0x%02X\n",rx_buf[i]);
    // }

    memcpy(pos,&rx_buf[14],6);// 0x0A 0x06
    memcpy(speed,&rx_buf[22],6);// 0x12 0x06
    memcpy(current,&rx_buf[30],6);// 0x1A 0x06

    return rx_len;
}




// #include "head.h"
#include "data_sensor_visual.h"
#include "filter.h"
#include <ctime>

bool serial_open;
std::queue<char> Queue;
uint32_t right_cnt;
uint32_t wrong_cnt;

sensor_brainco::stm32data stm32data_msg;
visualization_msgs::Marker marker[14];
std::mutex mtx;//互斥锁
std::mutex data_mtx;//stm32data_msg数据互斥锁

uint16_t raw_voldata[16];

std::atomic<bool> keepRunning(true);

int main( int argc, char** argv )
{
    uint8_t i=0;

    ros::init(argc, argv, "data_sensor_visual");
    ros::NodeHandle n_stm32,n_rviz,n_brainco_client;

    ros::Publisher stm32data_info_pub = n_stm32.advertise<sensor_brainco::stm32data>("/stm32data_info", 38);//传感器信息
    ros::Rate loop_rate1(100);
    WzSerialportPlus wzSerialportPlus;
    wzSerialportPlus.setReceiveCalback(serial_callback);

    // ros::Publisher marker_pub = n_rviz.advertise<visualization_msgs::Marker>("visualization_marker", 1);//rviz可视化
    // ros::Rate loop_rate2(100);


    std::thread serial_thread(serial_proj,wzSerialportPlus);
    std::thread data_process_thread(data_pro_proj);
    // std::thread visual_thread(visual_proj,marker_pub,loop_rate2);
    std::thread ros_thread(ros_to_sensor_proj,stm32data_info_pub,loop_rate1);
    
    // 设置信号处理函数
    signal(SIGINT, signalHandler);
    // 主线程等待用户输入ctrl+C
    // while (keepRunning) {
    //     // std::this_thread::sleep_for(std::chrono::seconds(1));
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // };

    data_process_thread.join();
    ros_thread.join();
    // visual_thread.join();
    // serial_thread.join();

    return 0;
}
//信号处理函数，当接收到SIGINT信号（由Ctrl+C产生）时，它会设置keepRunning为false。
void signalHandler(int signal) 
{
    // std::cout << std::endl << "main Caught signal: " << signal << std::endl;
    keepRunning = false;
    // exit(100);
}
//rviz maker可视化
void visual_proj(ros::Publisher pub,ros::Rate rosrate)
{
    uint8_t i,cnt=0;
    uint16_t color_cnt=0;

    uint32_t shape = visualization_msgs::Marker::CYLINDER;// CUBE SPHERE ARROW CYLINDER LINE_STRIP LINE_LIST CUBE_LIST SPHERE_LIST POINTS
    
    mtx.lock();
    ROS_INFO("visual_thread run success\r\n");
    mtx.unlock();
    for(i=0;i<14;i++)
    {
        marker[i].header.frame_id = "sensor";
        marker[i].header.stamp = ros::Time::now();
        marker[i].ns = "basic_shapes";
        marker[i].id = i;
        marker[i].type = shape;
        marker[i].action = visualization_msgs::Marker::MODIFY; // ADD MODIFY DELETE DELETEALL
        marker[i].lifetime = ros::Duration();
    }
    set_para(&marker[0],
    6.0, 0.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    0.0f, 1.0f, 0.0f, 1.0       //color.r g b a
    );
    set_para(&marker[10],
    6.0, 2.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 0.0f, 1.0       //color.r g b a
    );
    set_para(&marker[11],
    4.0, 2.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    0.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[12],
    2.0, 2.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[7],
    7.0, 5.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[6],
    7.0, 8.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[13],
    5.0, 5.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[5],
    5.0, 8.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[8],
    3.0, 5.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[4],
    3.0, 8.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[2],
    1.0, 5.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[3],
    1.0, 8.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[9],
    0.0, 0.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    set_para(&marker[1],
    -2.0, 0.0, 0.1,             //pose.position.x y z
    0.0, 0.0, 0.0, 1.0,         //pose.orientation.x y z w
    1.0, 1.0, 0.2,              //scale.x y z
    1.0f, 0.0f, 1.0f, 1.0       //color.r g b a
    );
    while (ros::ok()&&keepRunning)
    {
        GrayToPseColor_rainbow1(stm32data_msg.voltage[1],&marker[1].color.r,&marker[1].color.g,&marker[1].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[9],&marker[9].color.r,&marker[9].color.g,&marker[9].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[3],&marker[3].color.r,&marker[3].color.g,&marker[3].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[2],&marker[2].color.r,&marker[2].color.g,&marker[2].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[4],&marker[4].color.r,&marker[4].color.g,&marker[4].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[8],&marker[8].color.r,&marker[8].color.g,&marker[8].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[5],&marker[5].color.r,&marker[5].color.g,&marker[5].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[13],&marker[13].color.r,&marker[13].color.g,&marker[13].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[6],&marker[6].color.r,&marker[6].color.g,&marker[6].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[7],&marker[7].color.r,&marker[7].color.g,&marker[7].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[12],&marker[12].color.r,&marker[12].color.g,&marker[12].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[11],&marker[11].color.r,&marker[11].color.g,&marker[11].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[10],&marker[10].color.r,&marker[10].color.g,&marker[10].color.b);
        GrayToPseColor_rainbow1(stm32data_msg.voltage[0],&marker[0].color.r,&marker[0].color.g,&marker[0].color.b);
        
        
        // GrayToPseColor_rainbow1(color_cnt,&marker[3].color.r,&marker[3].color.g,&marker[3].color.b);
        // // if(cnt==3)
        // {
        //     if(color_cnt>=4095) color_cnt=0;
        //     else color_cnt+=4;
        // }
        
        // if(pub.getNumSubscribers() < 1)
        // {
        //     mtx.lock();
        //     ROS_WARN_ONCE("Please create a subscriber to the marker");
        //     mtx.unlock();
        // }
        // while (pub.getNumSubscribers() < 1)
        // {
        //     if (!ros::ok())
        //     {
        //         break;
        //     }
            
        //     // sleep(1);
        // }

        pub.publish(marker[cnt]);
        if(cnt>=13)cnt=0;
        else cnt++;

        // for(i=0;i<14;i++)marker_pub.publish(marker[i]);

        rosrate.sleep();
    }
}
//串口回调函数
void serial_callback(char* data,int length)
{
    int i=0;
    for(i=0;i<length;i++)
    {
        Queue.push(data[i]);
    }
    // mtx.lock();
    // ROS_INFO("queue size:%d\n",(int)Queue.size());
    // mtx.unlock();
}
//开启stm32串口接收
void serial_proj(WzSerialportPlus WzSerialport)
{
    if(WzSerialport.open("/dev/ttyUSB2",921600,1,8,'n'))
    {
        getchar();
        WzSerialport.close();
        std::cout << "serial end " << std::endl;
    }
    else 
    {
        ROS_INFO("stm32 Serial Open Error!!!!!!!!\n");
        exit(100);
    }
}
//处理stm32 传感器的电压 通讯帧
void data_pro_proj()
{
    uint8_t i,stat=0;
    uint8_t data_buff[32]={0};
    uint8_t crc_recv[2];
    while(keepRunning)
    {
        if(!Queue.empty())
        switch(stat)
        {
            case 0:
                if((uint8_t)Queue.front()==0xFF) stat=1;
                else stat=0;
                Queue.pop();
                break;
            case 1:
                if((uint8_t)Queue.front()==0xEE) stat=2;
                else stat=0;
                Queue.pop();
                break;
            case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10: 
                case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19: 
                case 20: case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28: 
                case 29: case 30: case 31: case 32: case 33: 
                data_buff[stat-2]=(uint8_t)Queue.front();
                Queue.pop();
                stat++;
                break;
            case 34:
                crc_recv[0]=(uint8_t)Queue.front();
                Queue.pop();
                stat=35;
                break;
            case 35:
                crc_recv[1]=(uint8_t)Queue.front();
                Queue.pop();
                stat=36;
                break;
            case 36:
                if((uint8_t)Queue.front()==0xDD) stat=37;
                else stat=0;
                Queue.pop();
                break;
            case 37:
                stat=0;
                if((uint8_t)Queue.front()==0xCC)
                {
                    Queue.pop();
                }
                else 
                {
                    Queue.pop();
                    break;
                }

                if(usMBCRC16(data_buff,32)==crc_recv[0]+(crc_recv[1]<<8))
                {
                    right_cnt++;
                    data_mtx.lock();
                    for(uint8_t i=0;i<14;i++)
                    {
                        raw_voldata[i]=data_buff[i*2]+(data_buff[i*2+1]<<8);
                    }
                    data_mtx.unlock();
                    // mtx.lock();
                    // ROS_INFO("data correct!\r\n");
                    // for(i=0;i<32;i++)
                    // {
                    //     printf("0x%2X ",data_buff[i]);
                    // }
                    // printf("\r\n");
                    // ROS_INFO("crc:0x%04X ",crc_recv[0]+(crc_recv[1]<<8));
                    // mtx.unlock();             
                }
                else
                {
                    wrong_cnt++;
                    if(right_cnt>200 && wrong_cnt>10)
                    {
                        mtx.lock();
                        ROS_INFO("right_cnt:%d wrong_cnt:%d error rate:%.1f%%",right_cnt,wrong_cnt,((double)wrong_cnt)/(wrong_cnt+right_cnt)*100);
                        mtx.unlock();
                    }
                }
                break;
        }
    }
}

// 将ros::Time转换为字符串的函数
std::string timeToString(const ros::Time& time) {
    std::stringstream ss;
    ss << time.sec << "_" << time.nsec; // 转换为秒和纳秒
    return ss.str();
}

#define FIR_BUF_NUM 10  //卷积系数的数量
void ros_to_sensor_proj(ros::Publisher pub,ros::Rate rosrate)
{
    // mtx.lock();
    // ROS_INFO("ros_thread run success\r\n");
    // mtx.unlock();

    std::vector<std::vector<double>> inputData(14, std::vector<double>(1)); // 创建一个包含14个向量的向量，每个向量初始大小为1
    std::vector<std::vector<double>> firFilteredData(14, std::vector<double>(1)); // 用于存储所有通道的FIR滤波数据
    std::vector<std::vector<double>> lagFilteredData(14, std::vector<double>(1)); // 用于存储所有通道的一阶滞后滤波数据
    
    // FIR滤波器系数
    std::vector<double> firCoefficients = {0.05, 0.1, 0.15, 0.1, 0.1, 0.05, 0.1, 0.15, 0.1, 0.1};
    double fir_buf[14][FIR_BUF_NUM]={0};

    // 一阶滞后滤波参数
    double alpha = 0.2;
    double lag_prevalue[14]={0};

    // 获取系统当前时间的Unix时间戳
    time_t now = time(NULL);

    struct tm *timeinfo = localtime(&now); // 将 time_t 转换为 tm 结构，本地时间

    char time_buffer[80]; // 创建一个字符数组作为缓冲区
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", timeinfo); // 使用特定格式填充缓冲区
    // std::cout << "Current time: " << time_buffer << std::endl; // 打印格式化的时间字符串
    std::string time_str = time_buffer;
    // std::string time_str = timeToString(ros::Time::now());
    
    // for(int i=0;i<14;i++)
    // {
    //     clearFileContent("/root/ros_noetic/18ws/src/new_code/ros-recv-stm32/sensor_brainco/record_data/filter_compare_data/inputData" + std::to_string(i) + ".txt");
    //     clearFileContent("/root/ros_noetic/18ws/src/new_code/ros-recv-stm32/sensor_brainco/record_data/filter_compare_data/firFilteredData" + std::to_string(i) + ".txt");
    //     clearFileContent("/root/ros_noetic/18ws/src/new_code/ros-recv-stm32/sensor_brainco/record_data/filter_compare_data/lagFilteredData" + std::to_string(i) + ".txt");
    // }
    // for(size_t i=1;i<=3;i++)
    // {
    //     clearFileContent("/root/ros_noetic/ros_senor/src/sensor_brainco/record_data/draw_data/voldata" + std::to_string(i) + time_str + ".txt");
    // }

    uint32_t cnt=0;
    uint16_t pre_vol[14]={0};
    double ini_aver[14]={0};

    // 循环开始前记录时间
    ros::Time loop_start_time = ros::Time::now();
    
    
    while (ros::ok() && keepRunning)
    {
        

        loop_start_time = ros::Time::now();
        data_mtx.lock();
        for(uint8_t i=0;i<14;i++)
        {
            inputData[i][0]=(double)raw_voldata[i];
        }
        data_mtx.unlock();

        // 应用FIR滤波
        for(int i=0;i<14;i++)
        {
            firFilteredData[i] = firFilter(inputData[i], firCoefficients,fir_buf[i],FIR_BUF_NUM);
        }
        
        // // 应用一阶滞后滤波
        // for(int i=0;i<14;i++)
        // {
        //     lagFilteredData[i] = firstOrderLagFilter(inputData[i], alpha, &lag_prevalue[i]);
        // }

        // // 保存数据到TXT文件
        // for(int i=0;i<14;i++)
        // {
        //     saveDataToFile("/root/ros_noetic/18ws/src/new_code/ros-recv-stm32/sensor_brainco/record_data/filter_compare_data/inputData" + std::to_string(i) + ".txt", inputData[i]);
        //     saveDataToFile("/root/ros_noetic/18ws/src/new_code/ros-recv-stm32/sensor_brainco/record_data/filter_compare_data/firFilteredData" + std::to_string(i) + ".txt", firFilteredData[i]);
        //     saveDataToFile("/root/ros_noetic/18ws/src/new_code/ros-recv-stm32/sensor_brainco/record_data/filter_compare_data/lagFilteredData" + std::to_string(i) + ".txt", lagFilteredData[i]);
        // }

        for(int i=0;i<14;i++)
        {
            stm32data_msg.voltage[i]=(uint16_t)firFilteredData[i][0];//进行fir滤波
        }
        // for(size_t i=1;i<=3;i++)
        // {
        //     saveDataToFile("/root/ros_noetic/18ws/src/new_code/ros-recv-stm32/sensor_brainco/record_data/draw_data/"+time_str+"vol_data[" 
        //     + std::to_string(i) + "]" + ".txt",firFilteredData[i]);
        // }
        if(cnt<200)
        {
            for(uint8_t i=0;i<14;i++)
            {
                ini_aver[i]+=stm32data_msg.voltage[i]/100.0;
            }
            
            cnt++;
        }
        else if(cnt==200)
        {
            for(uint8_t i=0;i<14;i++)
            {
                // stm32data_msg.initial_value[i]=stm32data_msg.voltage[i];
                stm32data_msg.initial_value[i]=ini_aver[i];
            }
            cnt++;
        }
        else
        {
            for(uint8_t i=0;i<14;i++)
            {
                stm32data_msg.diff[i]+=calculateRateOfChange(pre_vol[i],stm32data_msg.voltage[i]);
            }
        }

        for(uint8_t i=0;i<14;i++)
        {
            pre_vol[i]=stm32data_msg.voltage[i];
        }

        // 发布消息
		pub.publish(stm32data_msg);

        
        // 循环结束后记录时间
        ros::Time loop_end_time = ros::Time::now();
        
        // 计算循环所用的时间
        ros::Duration loop_duration = loop_end_time - loop_start_time;

        // // 输出循环所用的时间
        // ROS_INFO("Loop duration: %.6f seconds", loop_duration.toSec());

        // // 重置开始时间，以便下次循环使用
        // loop_start_time = ros::Time::now();

        // mtx.lock();
        // ROS_INFO("Publish stm32data_info:");
        // ROS_INFO("right_cnt:%d wrong_cnt:%d error rate:%.1f%%",right_cnt,wrong_cnt,((double)wrong_cnt)/(wrong_cnt+right_cnt)*100);
        // mtx.unlock();
        // 按照循环频率延时
        rosrate.sleep();
    }
}



#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <thread>

#include <atomic>
#include <csignal>

#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include <vector>
#include "draw_chart.h"
#include <ctime>
#include <cmath>
#include <numeric>

#include "threed_viz/robotiq3fctrl.h" 
#include "threed_viz/robotiq3f_feedback.h" 
#include "xela_server_ros/SensStream.h"
#include "sensor_brainco/stm32data.h"


threed_viz::robotiq3fctrl robotiq_Ctrl_msg;
threed_viz::robotiq3f_feedback robotiq3f_feedback_msg;
sensor_brainco::stm32data stm32data_msg;

// std::vector<double> SensorX(std::vector<double>(0));
// std::vector<double> SensorY(std::vector<double>(0));
// std::vector<double> SensorZ(std::vector<double>(0));
volatile uint8_t ctrl_command;
typedef struct
{
    double x;
    double y;
    double z;
    double len;
}SEN_COO;
double pearsonCorrelation(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("The size of x and y must be the same.");
    }

    double sum_x = std::accumulate(x.begin(), x.end(), 0.0);
    double sum_y = std::accumulate(y.begin(), y.end(), 0.0);
    double sum_xy = 0.0, sum_x2 = 0.0, sum_y2 = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        sum_xy += x[i] * y[i];
        sum_x2 += x[i] * x[i];
        sum_y2 += y[i] * y[i];
    }

    double mean_x = sum_x / x.size();
    double mean_y = sum_y / y.size();
    double numerator = sum_xy - (mean_x * sum_y);
    double denominator = std::sqrt((sum_x2 - (mean_x * mean_x * x.size())) * (sum_y2 - (mean_y * mean_y * y.size())));

    if (denominator == 0) {
        throw std::runtime_error("Denominator is zero, cannot divide by zero.");
    }

    return numerator / denominator;
}
int calculateDerivative(const std::vector<double>& data) {//导数全为正反馈1 全为负反馈-1 其他0
    std::vector<double> derivative(data.size() - 1);
    int overallSign = 0; // 默认值设为0，表示导数有正有负或全为零
    double dt = 1.0; // 假设采样间隔是1

    for (size_t i = 0; i < data.size() - 1; ++i) {
        double diff = (data[i + 1] - data[i]) / dt;
        derivative[i] = diff;

        // 根据第一个非零导数设置overallSign
        if (overallSign == 0 && diff != 0) {
            overallSign = (diff > 0) ? 1 : -1;
        }
        // 如果overallSign已经确定，检查是否有不同符号的导数出现
        else if ((overallSign == 1 && diff < 0) || (overallSign == -1 && diff > 0)) {
            overallSign = 0; // 发现不同符号，重置为0
            break;
        }
    }

    return overallSign;
}
std::vector<double> vecx;//实时计算相关系数缓存 一共缓存五个数据
std::vector<double> vecz;
std::vector<int> overallSign_x;//是否全为正或全为负数
std::vector<int> overallSign_z;
std::vector<double> correlation_xz;
std::vector<double> correlation_buffer;
// 定义一阶巴特沃斯高通滤波器
class ButterworthHighPassFilter {
public:
    ButterworthHighPassFilter(double cutoffFreq, double sampleRate)
        : cutoffFreq(cutoffFreq), sampleRate(sampleRate), prevInput(0.0), prevOutput(0.0) {
        double RC = 1.0 / (cutoffFreq * 2 * M_PI);
        double dt = 1.0 / sampleRate;
        alpha = dt / (RC + dt);
    }

    double process(double input) {
        double output = alpha * (prevOutput + input - prevInput);
        prevInput = input;
        prevOutput = output;
        return output;
    }

private:
    double cutoffFreq;
    double sampleRate;
    double alpha;
    double prevInput;
    double prevOutput;
};
class BiquadBandFilter {
public:
    BiquadBandFilter(double lowCutoff, double highCutoff,double sampleRate )
        :  lowCutoff(lowCutoff), highCutoff(highCutoff),sampleRate(sampleRate) {
        calculateCoefficients();
    }

    double process(double input) {
        double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }

    void updateCutoffFrequencies(double lowCutoff, double highCutoff) {
        this->lowCutoff = lowCutoff;
        this->highCutoff = highCutoff;
        calculateCoefficients();
    }

private:
    double sampleRate;
    double lowCutoff;
    double highCutoff;

    double a1, a2, b0, b1, b2;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    void calculateCoefficients() {
        double omega = 2.0 * M_PI * sqrt(lowCutoff * highCutoff) / sampleRate;
        double bw = 2.0 * M_PI * (highCutoff - lowCutoff) / sampleRate;
        double alpha = sin(omega) * sinh(log(2.0) / 2.0 * bw * omega / sin(omega));

        double cosw0 = cos(omega);
        double a0 = 1.0 + alpha;
        
        b0 = alpha / a0;
        b1 = 0;
        b2 = -alpha / a0;
        a1 = -2.0 * cosw0 / a0;
        a2 = (1.0 - alpha) / a0;
    }
};

class ButterworthLowPassFilter {
private:
    double gain_;
    double prev_output_;

public:
    // 构造函数
    ButterworthLowPassFilter(double cutoff_freq, double sample_freq, int order = 1)
    {
        // 计算滤波器的增益，这里简化为使用截止频率和采样频率
        double normalized_cutoff_freq = cutoff_freq / (sample_freq / 2); // 归一化截止频率
        gain_ = 1 / (1 + std::tan(M_PI * normalized_cutoff_freq / order));
        prev_output_ = 0.0;
    }

    // 应用巴特沃斯低通滤波器
    double process(double input) {
        // 使用一阶滤波器的递归公式
        double output = gain_ * (input + prev_output_);
        prev_output_ = output;
        return output;
    }
};

SEN_COO sen_coo[16];
SEN_COO sen_all;
//巴特沃斯高通滤波
ButterworthHighPassFilter filter_x(5.0, 100.0);
ButterworthHighPassFilter filter_y(5.0, 100.0);
ButterworthHighPassFilter filter_z(5.0, 100.0);
//带通滤波器
// BiquadBandFilter filter_x(15.0,50.0, 100.0);//外界机械扰动在1hz～5hz内明显
// BiquadBandFilter filter_y(15.0,50.0, 100.0);
// BiquadBandFilter filter_z(15.0,50.0, 100.0);
//巴特沃斯低通滤波
// ButterworthLowPassFilter filter_x(2.0, 100.0);
// ButterworthLowPassFilter filter_y(2.0, 100.0);
// ButterworthLowPassFilter filter_z(5.0, 100.0);
double sen_filter_x;
double sen_filter_y;
double sen_filter_z;

ButterworthHighPassFilter filter_3(5.0, 100.0);
ButterworthHighPassFilter filter_5(5.0, 100.0);
ButterworthHighPassFilter filter_10(5.0, 100.0);
ButterworthHighPassFilter filter_11(5.0, 100.0);
double stm32filter_3;
double stm32filter_5;
double stm32filter_10;
double stm32filter_11;

std::string savepath = "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/";

uint32_t xsen_record_cnt = 0;
uint8_t slip_flag = 0;
void stm32sen_callback(const sensor_brainco::stm32data::ConstPtr& msg)
{
    memcpy(&stm32data_msg,msg.get(),128);
    // ROS_INFO("10:%4d,11:%4d,3:%4d,5:%4d",stm32data_msg.voltage[10],stm32data_msg.voltage[11],stm32data_msg.voltage[3],stm32data_msg.voltage[5]);
    stm32filter_3=filter_3.process(stm32data_msg.voltage[3]);
    stm32filter_5=filter_5.process(stm32data_msg.voltage[5]);
    stm32filter_10=filter_10.process(stm32data_msg.voltage[10]);
    stm32filter_11=filter_11.process(stm32data_msg.voltage[11]);
    
    // saveDataToFile(savepath+"stm32raw3.txt", stm32data_msg.voltage[3]);
    // saveDataToFile(savepath+"stm32raw5.txt", stm32data_msg.voltage[5]);
    // saveDataToFile(savepath+"stm32raw10.txt", stm32data_msg.voltage[10]);
    // saveDataToFile(savepath+"stm32raw11.txt", stm32data_msg.voltage[11]);

    // saveDataToFile(savepath+"stm32fil3.txt", stm32filter_3);
    // saveDataToFile(savepath+"stm32fil5.txt", stm32filter_5);
    // saveDataToFile(savepath+"stm32fil10.txt", stm32filter_10);
    // saveDataToFile(savepath+"stm32fil11.txt", stm32filter_11);

}
void xsen_callback(const xela_server_ros::SensStream::ConstPtr& msg)
{
    xsen_record_cnt++;
    // 获取系统当前时间的Unix时间戳
    time_t now = time(NULL);

    struct tm *timeinfo = localtime(&now); // 将 time_t 转换为 tm 结构，本地时间

    char time_buffer[80]; // 创建一个字符数组作为缓冲区
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", timeinfo); // 使用特定格式填充缓冲区
    // std::cout << "Current time: " << time_buffer << std::endl; // 打印格式化的时间字符串
    std::string time_str = time_buffer;
    // std::string time_str = timeToString(ros::Time::now());

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
        sen_filter_x=filter_x.process(sen_all.x);//滤波
        sen_filter_y=filter_y.process(sen_all.y);
        sen_filter_z=filter_z.process(sen_all.z);
        saveDataToFile(savepath+"xela_x.txt", sen_all.x);//存到txt
        saveDataToFile(savepath+"xela_y.txt", sen_all.y);
        saveDataToFile(savepath+"xela_z.txt", sen_all.z);
        saveDataToFile(savepath+"filter_x.txt", sen_filter_x);
        saveDataToFile(savepath+"filter_y.txt", sen_filter_y);
        saveDataToFile(savepath+"filter_z.txt", sen_filter_z);

        for(uint8_t i=0;i<16;i++)
        {
            saveDataToFile(savepath+"xela"+std::to_string(i)+"_x.txt", msg->sensors[0].forces[i].x);//存到txt
            saveDataToFile(savepath+"xela"+std::to_string(i)+"_y.txt", msg->sensors[0].forces[i].y);
            saveDataToFile(savepath+"xela"+std::to_string(i)+"_z.txt", msg->sensors[0].forces[i].z);
        }
        sen_all.len=std::sqrt(sen_all.x * sen_all.x + sen_all.y * sen_all.y + sen_all.z * sen_all.z);
        // ROS_INFO("ALL X: %f, Y: %f, Z: %f,len: %f",sen_all.x ,sen_all.y ,sen_all.z,sen_all.len);

        static uint8_t run_stat=0;
        if(vecx.size()<5)
        {
            vecx.push_back(sen_all.x);
            vecz.push_back(sen_all.z);
        }
        // else if(vecx.size()==4)
        // {
        //     vecx.push_back(sen_all.x);
        //     vecz.push_back(sen_all.z);
        //     correlation_xz.push_back(pearsonCorrelation(vecx, vecz));//计算原数据首位的相关系数
        // }
        else
        {
            vecx.erase(vecx.begin());//移除缓存最前面的数据
            vecz.erase(vecz.begin());

            vecx.push_back(sen_all.x);//添加新数据到缓存
            vecz.push_back(sen_all.z);

            overallSign_x.push_back(calculateDerivative(vecx));//计算x轴overallSign
            overallSign_z.push_back(calculateDerivative(vecz));//计算z轴overallSign
            correlation_xz.push_back(pearsonCorrelation(vecx, vecz));//计算原数据首位的相关系数

            switch(run_stat)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                    if(correlation_xz[0]<-0.8)
                    {
                        run_stat++;
                    }
                    else
                    {
                        run_stat=0;
                    }
                    correlation_xz.erase(correlation_xz.begin());
                    // cnt++;
                    if(run_stat!=10)break;
                case 10:
                    // if(overallSign_z[xsen_record_cnt-5]<=0 && overallSign_x[xsen_record_cnt-5]>0)
                    // if(overallSign_x[xsen_record_cnt-5]>0)
                    {
                        ROS_INFO("slip %d",xsen_record_cnt);
                        slip_flag=1;
                        // std::cout << "get" << xsen_record_cnt << std::endl;
                        // std::cout << "dataZ Derivative:" << overallSign_z[xsen_record_cnt-5] << std::endl;
                        // std::cout << "dataX Derivative:" << overallSign_x[xsen_record_cnt-5] << std::endl;
                    }
                    run_stat--;
                    break;
            }
        }

        
    }
}
void robotiq3f_fb_callback(const threed_viz::robotiq3f_feedback::ConstPtr& msg)
{
    robotiq3f_feedback_msg.A_position=msg->A_position;
    robotiq3f_feedback_msg.A_current =msg->A_current;
    robotiq3f_feedback_msg.B_position=msg->B_position;
    robotiq3f_feedback_msg.B_current =msg->B_current;
    robotiq3f_feedback_msg.C_position=msg->C_position;
    robotiq3f_feedback_msg.C_current =msg->C_current;
    
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
        robotiq_Ctrl_msg.stop=0;

        // 按照循环频率延时
        rosrate.sleep();
    }
}
void robotiq_Ctrl_Once(uint8_t pos,uint8_t spe,uint8_t cur)
{
    if(pos>105)pos=105;
    robotiq_Ctrl_msg.position=pos;
    robotiq_Ctrl_msg.speed=spe;
    robotiq_Ctrl_msg.force=cur;
    robotiq_Ctrl_msg.command=1;
}
void robotiq_stop()
{
    robotiq_Ctrl_msg.command=1;
    robotiq_Ctrl_msg.stop=1;
}
int camera_proj()
{
    cv::VideoCapture capture(6); // 打开摄像头
    if (!capture.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }

    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    double width=1280;
    double height=720;
    double fps = capture.get(cv::CAP_PROP_FPS);

    // 定义MP4视频文件的路径
    std::string videoFile = "/home/galaxy/Desktop/Xela_ws/src/threed_viz/video/output.mp4";
    // 定义视频的四字符代码为H.264编码器
    int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    // 创建VideoWriter对象，指定MP4格式和H.264编码器
    cv::VideoWriter videoWriter;
    videoWriter.open(videoFile, fourcc, fps, cv::Size(width, height), true);
    if (!videoWriter.isOpened()) {
        std::cerr << "无法创建视频文件" << std::endl;
        return -1;
    }

    cv::Mat frame;
    for(int i=0;i<300;i++)
    // while (1) 
    {
        capture.read(frame); // 读取帧
        if (frame.empty()) {
            std::cerr << "无法读取帧" << std::endl;
            break;
        }
        // 写入帧到MP4视频文件
        videoWriter.write(frame);

        cv::imshow("Video", frame); // 显示帧

        if (cv::waitKey(30) >= 0) break;
        if (!ros::ok())break;
    }

    capture.release(); // 释放摄像头
    videoWriter.release(); // 释放VideoWriter
    cv::destroyAllWindows(); // 销毁所有窗口
    std::cerr << "已关闭摄像头" << std::endl;
    return 0;
}

void main_proj(ros::Publisher pub)//向brainco请求反馈数据,然后发布反馈数据fb
{
    uint8_t run_stat=0;
    sleep(1);
    
    if(robotiq3f_feedback_msg.A_position>20)
    {
        robotiq_Ctrl_Once(0,250,0);
        sleep(3);
    }
    robotiq_Ctrl_Once(105,0,0);//合拢
    ROS_INFO("motion");
    uint8_t xsen_touch_flag = 0;
    uint8_t stm32_touch_flag = 0;
    
    uint8_t touch_pos = 0 ;//接触时位置
    uint8_t grasp_pos = 0 ;//抓取的位置增量
    double record_force[4] = {0};
    double diff_force = 0;
    while(ros::ok())
    { 
        switch(run_stat)
        {
            case 0:
                if(sen_filter_z>0.05)
                {
                    if(xsen_touch_flag==0)
                    ROS_INFO("xsen_touch_flag");
                    xsen_touch_flag = 1;
                }
                    
                if((stm32filter_3<-7)||(stm32filter_5<-7)||(stm32filter_10<-7)||(stm32filter_11<-7))
                {
                    if(stm32_touch_flag==0)
                    ROS_INFO("stm32_touch_flag");
                    stm32_touch_flag = 1;
                }
                    

                if( xsen_touch_flag && stm32_touch_flag )//三个手指都接触
                {
                    ROS_INFO("touch");
                    touch_pos = robotiq3f_feedback_msg.A_position;
                    // robotiq_stop();
                    // robotiq_Ctrl_Once(105,0,0);//控力
                    
                    run_stat=1;//进行顺应性检测

                    // run_stat=10;//不进行顺应性检测
                    // robotiq_Ctrl_Once(touch_pos+5,50,0);//手动设置抓握位置

                    
                    ROS_INFO("run_stat:%d,xsen_cnt:%d",run_stat,xsen_record_cnt);
                    
                }
                break;
            case 1:
                robotiq_Ctrl_Once(touch_pos,50,0);//停止闭合
                usleep(1000*800);
                record_force[0] = sen_all.z;
                robotiq_Ctrl_Once(touch_pos+1,0,1);
                usleep(1000*500);
                record_force[1] = sen_all.z;
                robotiq_Ctrl_Once(touch_pos+2,0,1);
                usleep(1000*500);
                record_force[2] = sen_all.z;

                diff_force = record_force[2] - record_force[0];
                
                ROS_INFO("record_force: %.3f %.3f %.3f",record_force[0],record_force[1],record_force[2]);
                ROS_INFO("diff_force: %.3f",diff_force);
                // ROS_INFO("compliance:%f mm/N",3.0/diff_force);
                
                // run_stat=3;//松开结束
                run_stat=2;//继续夹持
                ROS_INFO("Catch");
                ROS_INFO("run_stat:%d,xsen_cnt:%d",run_stat,xsen_record_cnt);

                break;
            
            case 2:
                grasp_pos=(uint8_t)(diff_force/4);
                if(grasp_pos>4)grasp_pos=4;

                robotiq_Ctrl_Once(touch_pos+grasp_pos,20,0);
                // usleep(1000*10000);
                // for(size_t i=0;i<50;i++)
                // {
                //     if(slip_flag)
                //     {
                //         grasp_pos+=2;
                //         robotiq_Ctrl_Once(touch_pos+grasp_pos,20,0);
                //         ROS_INFO("plus press");
                //         usleep(1000*100);
                //         slip_flag=0;
                //     }
                //     usleep(1000*100);
                    
                // }
                sleep(5);
                ROS_INFO("down");
                sleep(5);
                // for(size_t i=0;i<50;i++)
                // {
                //     if(slip_flag)
                //     {
                //         grasp_pos+=2;
                //         robotiq_Ctrl_Once(touch_pos+grasp_pos,20,0);
                //         ROS_INFO("plus press");
                //         usleep(1000*100);
                //         slip_flag=0;
                //     }
                //     usleep(1000*100);
                // }
                run_stat=3;//松开结束
                
                break;
            case 3:
                robotiq_Ctrl_Once(0,20,0);
                ROS_INFO("release");
                ROS_INFO("run_stat:%d,xsen_cnt:%d",run_stat,xsen_record_cnt);
                run_stat=255;
                break;

            case 10:
                sleep(12);
                ROS_INFO("down");
                run_stat=11;
                ROS_INFO("run_stat:%d,xsen_cnt:%d",run_stat,xsen_record_cnt);
                break;

            case 11:
                sleep(10);
                robotiq_Ctrl_Once(0,20,0);
                ROS_INFO("release");
                run_stat=12;
                ROS_INFO("run_stat:%d,xsen_cnt:%d",run_stat,xsen_record_cnt);
                break;

            case 4:
                break;

            case 5:
                break;


            default:
                break;
                
        }
        // usleep(100*1);
    }

}
int main( int argc, char** argv )
{
    ros::init(argc, argv, "motion_control");

    ros::NodeHandle n1,n2,n3;
    clearFileContent(savepath+"xela_x.txt");
    clearFileContent(savepath+"xela_y.txt");
    clearFileContent(savepath+"xela_z.txt");
    clearFileContent(savepath+"filter_x.txt");
    clearFileContent(savepath+"filter_y.txt");
    clearFileContent(savepath+"filter_z.txt");

    // clearFileContent(savepath+"stm32raw3.txt");
    // clearFileContent(savepath+"stm32raw5.txt");
    // clearFileContent(savepath+"stm32raw10.txt");
    // clearFileContent(savepath+"stm32raw11.txt");
    // clearFileContent(savepath+"stm32fil3.txt");
    // clearFileContent(savepath+"stm32fil5.txt");
    // clearFileContent(savepath+"stm32fil10.txt");
    // clearFileContent(savepath+"stm32fil11.txt");
    for(uint8_t i=0;i<16;i++)
    {
        clearFileContent(savepath+"xela"+std::to_string(i)+"_x.txt");
        clearFileContent(savepath+"xela"+std::to_string(i)+"_y.txt");
        clearFileContent(savepath+"xela"+std::to_string(i)+"_z.txt");
    }

    ros::Subscriber sub1 = n1.subscribe("xServTopic", 1000, xsen_callback);
    ros::Subscriber sub2 = n2.subscribe("stm32data_info", 1000, stm32sen_callback);
    ros::Subscriber sub3 = n3.subscribe("robotiq3f_feedback", 1000, robotiq3f_fb_callback);
    ros::Publisher robotiq_Ctrl_info_pub = n3.advertise<threed_viz::robotiq3fctrl>("/robotiq3fctrl", 10);
    ros::Rate loop_rate_pub(200);
    
    std::thread ros_robotiq_Ctrl_pub_thread(robotiq_Ctrl_pub,robotiq_Ctrl_info_pub,loop_rate_pub); 
    
    std::thread main_proj_thread(main_proj,robotiq_Ctrl_info_pub);
    
    std::thread camera_proj_thread(camera_proj);

    ros::spin();
    camera_proj_thread.join();
}
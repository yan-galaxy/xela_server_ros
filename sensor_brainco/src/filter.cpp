#include <unistd.h> 
#include <fstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <sys/stat.h> // 包含用于文件状态的函数
#include <cstring> // 包含此头文件以使用 strerror 函数

// 计算变化率的函数
double calculateRateOfChange(double oldValue, double newValue) 
{
    // 计算变化量
    double change = newValue - oldValue;

    // if(change>=10)
    //     return change;
    // else if(change<10 && change>-13)
    //     return 0;
    // else
    //     return change*2.5;


    // 计算变化率
    // double rate = (change / oldValue) * 100.0; // 百分比变化率

    // 如果需要绝对变化率，可以返回 change / timeElapsed;

    return change;
}


// FIR滤波函数
std::vector<double> firFilter(const std::vector<double>& data, const std::vector<double>& coefficients,double * fir_buf,uint8_t fir_bufnum)
{
    std::vector<double> filteredData;
    fir_buf[fir_bufnum-1]=data[0];
    double sum = 0.0;
    for (size_t j = 0; j < coefficients.size(); ++j) 
    {
        sum += coefficients[j] * fir_buf[fir_bufnum-1 - j];
    }
    filteredData.push_back(sum);
    for(int i=0;i<(fir_bufnum-1);i++)
    {
        fir_buf[i]=fir_buf[i+1];
    }
    
    return filteredData;
}
// 一阶滞后滤波函数
std::vector<double> firstOrderLagFilter(const std::vector<double>& data, double alpha,double* previousValue) 
{
    std::vector<double> filteredData;
    // static double previousValue = 0.0;
    // for (double value : data) {
    //     double currentValue = alpha * value + (1 - alpha) * previousValue;
    //     filteredData.push_back(currentValue);
    //     previousValue = currentValue;
    // }
    // for (double value : data) {
        double currentValue = alpha * data[0] + (1 - alpha) * *previousValue;
        filteredData.push_back(currentValue);
        *previousValue = currentValue;
    // }
    return filteredData;
}

// 保存数据到新建的TXT文件

void saveDataToFile_new(const std::string& filename, const std::vector<double>& data) {
    std::ofstream outFile(filename, std::ios::out);

    // 检查文件是否成功打开
    if (!outFile) {
        std::cerr << "无法打开或创建文件: " << filename << std::endl;
        // 可以进一步检查 errno 来获取错误原因
        if (errno == EACCES) {
            std::cerr << "权限不足，无法创建文件。" << std::endl;
        } else {
            std::cerr << "错误代码: " << errno << std::endl;
        }
        return;
    }

    try {
        for (const auto& value : data) {
            outFile << value << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "写入文件时发生异常: " << e.what() << std::endl;
    }
    outFile.close(); // 关闭文件
    // 关闭文件
    if (outFile.is_open()) {
        std::cerr << "无法正确关闭文件: " << filename << std::endl;
    }
}
// void saveDataToFile_new(const std::string& filename, const std::vector<double>& data) {
//     std::ofstream outFile(filename);
//     for (double value : data) {
//         outFile << value << std::endl;
//     }
//     outFile.close();
// }

// 保存数据到TXT文件
bool saveDataToFile(const std::string& filename, const std::vector<double>& data) 
{
    std::ofstream outFile(filename, std::ios::app); // 以追加模式打开文件
    if (!outFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false; // 文件打开失败
    }

    // for (double value : data) {
        // outFile << value << std::endl;
        outFile << data[0] << std::endl;
        if (outFile.fail()) {
            std::cerr << "写入文件时发生错误: " << filename << std::endl;
            outFile.close();
            return false; // 数据写入失败
        }
    // }

    outFile.close();
    return true; // 数据写入成功
}

// 以输出模式打开文件，这将清空文件内容
void clearFileContent(const std::string& filename) 
{
    std::ofstream outFile(filename, std::ios::trunc);
    if (!outFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }
    // 不需要写入任何内容，文件打开后内容已被清空
    // 可以选择写入一个空行或者不写任何内容
    outFile.close();
}
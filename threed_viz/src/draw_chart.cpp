#include <unistd.h> 
#include <fstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <sys/stat.h> // 包含用于文件状态的函数
#include <cstring> // 包含此头文件以使用 strerror 函数

// 保存数据到TXT文件
bool saveDataToFile(const std::string& filename, double data) 
{
    std::ofstream outFile(filename, std::ios::app); // 以追加模式打开文件
    if (!outFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false; // 文件打开失败
    }

    // for (double value : data) {
        // outFile << value << std::endl;
        outFile << data << std::endl;
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
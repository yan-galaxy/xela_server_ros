#ifndef _FILTER_H
#define _FILTER_H



double calculateRateOfChange(double oldValue, double newValue);
// FIR滤波函数
std::vector<double> firFilter(const std::vector<double>& data, const std::vector<double>& coefficients,double * fir_buf,uint8_t fir_bufnum);
// 一阶滞后滤波函数
std::vector<double> firstOrderLagFilter(const std::vector<double>& data, double alpha,double* previousValue);
// 保存数据到新建的TXT文件
void saveDataToFile_new(const std::string& filename, const std::vector<double>& data);
// 保存数据到TXT文件
bool saveDataToFile(const std::string& filename, const std::vector<double>& data);
// 以输出模式打开文件，这将清空文件内容
void clearFileContent(const std::string& filename);

#endif
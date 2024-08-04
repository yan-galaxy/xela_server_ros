#ifndef _DRAW_CHART_H
#define _DRAW_CHART_H


// 保存数据到TXT文件
bool saveDataToFile(const std::string& filename, double data);
// 以输出模式打开文件，这将清空文件内容
void clearFileContent(const std::string& filename);

#endif
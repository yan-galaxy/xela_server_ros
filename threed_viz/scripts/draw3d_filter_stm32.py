#!/usr/bin/env python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

# 定义一个函数来读取数据
def read_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # 跳过文件头部可能存在的非数据行
        # 假设第一行是文件名或注释，从第二行开始读取数据
        return [float(line.strip()) for line in lines[1:]]


file_paths = [
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32raw3.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32raw5.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32raw10.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32raw11.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32fil3.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32fil5.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32fil10.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/stm32fil11.txt"
] 
# 读取数据
raw3, raw5, raw10, raw11, fil3, fil5, fil10, fil11 = [read_data(path) for path in file_paths]

x = np.arange(len(raw3))
y1, y2, y3, y4, y5, y6, y7, y8 = np.full_like(x, 1), np.full_like(x, 2), np.full_like(x, 3), np.full_like(x, 4), np.full_like(x, 5), np.full_like(x, 6), np.full_like(x, 7), np.full_like(x, 8)

# 创建一个新的图和子图
fig = plt.figure(figsize=(12, 6))
fig.canvas.manager.set_window_title("Raw and Filtered Data")

# 第一个子图：原始数据
ax1 = fig.add_subplot(121, projection='3d')

# 定义每组数据的颜色  可以是颜色名称、十六进制颜色代码或颜色元组  
# FFA500 浅橙色      darkorange
# ADD8E6 lightblue 浅蓝色  87CEEB skyblue   6495ED cornflowerblue
# 90EE90 lightgreen 浅绿色    66CD00
# FFB6C1 lightpink 浅红色
colors = ['red', 'lightpink', 'cornflowerblue', 'skyblue', '#FFA500','darkorange', 'lightgreen', 'green']

# 绘制三维线图，这里我们使用不同的y坐标和颜色来区分不同的数据集
ax1.plot(x, y1, raw3, c='red', label='raw3')
ax1.plot(x, y2, raw5, c='cornflowerblue', label='raw5')
ax1.plot(x, y3, raw10, c='darkorange', label='raw10')
ax1.plot(x, y4, raw11, c='green', label='raw11')

# 设置图例
ax1.legend()

# 设置坐标轴标签
ax1.set_xlabel('Time/10ms')
ax1.set_ylabel('Group')
ax1.set_zlabel('voltage')
ax1.set_title("Raw Data")


# 第二个子图：滤波后的数据
ax2 = fig.add_subplot(122, projection='3d')

# 绘制滤波后的数据
ax2.plot(x, y1, fil3, c='lightpink', label='fil3')
ax2.plot(x, y2, fil5, c='skyblue', label='fil5')
ax2.plot(x, y3, fil10, c='#FFA500', label='fil10')
ax2.plot(x, y4, fil11, c='lightgreen', label='fil11')

# 设置图例
ax2.legend()

# 设置坐标轴标签
ax2.set_xlabel('Time/10ms')
ax2.set_ylabel('Group')
ax2.set_zlabel('voltage')
ax2.set_title("Filtered Data")

# 显示图形
plt.tight_layout()
plt.show()
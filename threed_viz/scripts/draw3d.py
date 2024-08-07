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

if len(sys.argv) > 2 and sys.argv[2] == 'f':#第0个参数是程序本身 第1个参数是文件夹名 有第2个参数f是显示滤波图形
   # 指定文件路径
    file_paths = [
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_x.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_y.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_z.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/filter_x.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/filter_y.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/filter_z.txt"
    ] 
elif len(sys.argv) >1 :#一个输入参数
    file_paths = [
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_x.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_y.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_z.txt"
    ] 
else:#没有输入参数 原路径
    file_paths = [
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_x.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_y.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_z.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/filter_x.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/filter_y.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/filter_z.txt"
    ] 
# 读取数据
datax, datay, dataz, fdatax, fdatay, fdataz = [read_data(path) for path in file_paths]


# 创建一个新的图和子图
fig = plt.figure(figsize=(12, 6))
fig.canvas.manager.set_window_title("Raw and Filtered Data")

# 第一个子图：原始数据
ax1 = fig.add_subplot(121, projection='3d')
x = np.arange(len(datax))
y1, y2, y3, y4, y5 = np.full_like(x, 1), np.full_like(x, 2), np.full_like(x, 3), np.full_like(x, 4), np.full_like(x, 5)


# 定义每组数据的颜色  可以是颜色名称、十六进制颜色代码或颜色元组  
# FFA500 浅橙色  Light Orange
# ADD8E6 lightblue 浅蓝色  87CEEB skyblue   6495ED cornflowerblue
# 90EE90 lightgreen 浅绿色    66CD00
# FFB6C1 lightpink 浅红色
colors = ['red', 'lightpink', 'cornflowerblue', 'skyblue', 'lightorange','darkorange', 'lightgreen', 'green']

# 绘制三维线图，这里我们使用不同的y坐标和颜色来区分不同的数据集
ax1.plot(x, y1, datax, c='green', label='shear(raw)')
ax1.plot(x, y2, dataz, c='cornflowerblue', label='normal(raw)')

# 设置图例
ax1.legend()

# 设置坐标轴标签
ax1.set_xlabel('Time/10ms')
ax1.set_ylabel('Group')
ax1.set_zlabel('Force/N')
ax1.set_title("Raw Data")

if sys.argv[2] == 'f':
    # 第二个子图：滤波后的数据
    ax2 = fig.add_subplot(122, projection='3d')

    # 绘制滤波后的数据
    ax2.plot(x, y1, fdatax, c='#66CD00', label='shear(filtered)')#比浅绿色稍微深一点点
    ax2.plot(x, y2, fdataz, c='skyblue', label='normal(filtered)')

    # 设置图例
    ax2.legend()

    # 设置坐标轴标签
    ax2.set_xlabel('Time/10ms')
    ax2.set_ylabel('Group')
    ax2.set_zlabel('Force/N')
    ax2.set_title("Filtered Data")

# 显示图形
plt.tight_layout()
plt.show()
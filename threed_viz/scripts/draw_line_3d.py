#!/usr/bin/env python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

def highpass_filter(data, fs, cutoff):
    """
    对数据应用一阶巴特沃斯高通滤波器
    
    参数:
    - data: 要滤波的原始数据
    - fs: 采样率
    - cutoff: 高通滤波器的截止频率
    
    返回:
    - 滤波后的数据
    """
    # 计算滤波器参数
    RC = 1.0 / (cutoff * 2 * np.pi)
    dt = 1.0 / fs
    alpha = dt / (RC + dt)

    # 初始化滤波后的数据
    filtered_data = np.zeros_like(data)

    # 初始条件
    filtered_data[0] = data[0]

    # 应用高通滤波器
    for i in range(1, len(data)):
        filtered_data[i] = alpha * (filtered_data[i - 1] + data[i] - data[i - 1])
    
    return filtered_data

# 定义一个函数来读取数据
def read_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # 跳过文件头部可能存在的非数据行
        # 假设第一行是文件名或注释，从第二行开始读取数据
        return [float(line.strip()) for line in lines[1:]]

if len(sys.argv) > 1:
   # 指定文件路径
    file_paths = [
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_x.txt",#5是抓紧的没有滑动，并且上升速度为1
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_y.txt",#8是一直在滑动，上升速度为3
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/"+ sys.argv[1] +"/xela_z.txt"
    ] 
else:#没有输入参数 原路径
    file_paths = [
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_x.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_y.txt",
        "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_z.txt"
    ] 
# 读取数据
datax, datay, dataz = [read_data(path) for path in file_paths]

# 应用滤波器
filtered_dataz = highpass_filter(dataz, fs=100, cutoff=5)
filtered_datax = highpass_filter(datax, fs=100, cutoff=5)

# 创建一个新的图和子图
fig = plt.figure()
fig.canvas.manager.set_window_title("raw")
ax = fig.add_subplot(111, projection='3d')

# 为了绘制三维图，我们需要x, y, z坐标
# 这里我们使用数据索引作为x坐标，不同的y坐标来区分数据组
x = np.arange(len(datax))
y1, y2, y3,y4 ,y5 = np.full_like(x, 1), np.full_like(x, 2), np.full_like(x, 2), np.full_like(x, 4), np.full_like(x, 5)


# 定义每组数据的颜色  可以是颜色名称、十六进制颜色代码或颜色元组  
# FFA500 浅橙色  
# ADD8E6 lightblue 浅蓝色  87CEEB skyblue   6495ED cornflowerblue
# 90EE90 lightgreen 浅绿色    66CD00
# FFB6C1 lightpink 浅红色
colors = ['red','lightpink','cornflowerblue','skyblue','#FFA500','lightgreen','green']

# 绘制三维散点图，这里我们使用不同的y坐标和颜色来区分不同的数据集
scatter1 = ax.plot(x, y1, datax,c='green', label='shear(raw)')#X   ax.plot  ax.scatter
# scatter2 = ax.scatter(x, y2, filtered_datax,c='lightgreen', label='filtered_shear')#Y
scatter3 = ax.plot(x, y3, dataz,c='cornflowerblue', label='normal(raw)')#Z

# 设置图例
ax.legend()

# 设置坐标轴标签
ax.set_xlabel('Time/10ms')
ax.set_ylabel('Group')
ax.set_zlabel('Force/N')

# 显示图形
plt.show(block=False)




# 创建第二个图和子图，用于显示滤波后的数据
fig2 = plt.figure()
fig2.canvas.manager.set_window_title("filtered")
ax2 = fig2.add_subplot(111, projection='3d')

# 绘制滤波后的数据
ax2.plot(x, y1, filtered_datax, c='#66CD00', label='shear(filtered)')#比浅绿色稍微深一点点
ax2.plot(x, y3, filtered_dataz, c='skyblue', label='shear(filtered)')

# 设置图例
ax2.legend()

# 设置坐标轴标签
ax2.set_xlabel('Time/10ms')
ax2.set_ylabel('Group')
ax2.set_zlabel('Force/N')

# 显示滤波后的图形
plt.show()
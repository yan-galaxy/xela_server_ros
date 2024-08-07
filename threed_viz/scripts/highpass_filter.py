import numpy as np
import matplotlib.pyplot as plt

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

# 读取数据文件
data_file = '/home/galaxy/Desktop/Xela_ws/src/threed_viz/storage_data/40/xela_x.txt'
data = np.loadtxt(data_file)

# 采样率和滤波器参数
fs = 100  # 采样率为100Hz
cutoff = 5  # 高通滤波器的截止频率为5Hz

# 应用滤波器
filtered_data = highpass_filter(data, fs, cutoff)

# 绘制原始数据和滤波后数据
plt.figure(figsize=(14, 6))

# 绘制原始数据
plt.subplot(2, 1, 1)
plt.plot(data, label='raw data')
plt.title('raw data')
plt.xlabel('sample point')
plt.ylabel('value')
plt.legend()

# 绘制滤波后数据
plt.subplot(2, 1, 2)
plt.plot(filtered_data, label='filtered data', color='red')
plt.title('filtered data (cut off freq='+str(cutoff)+'Hz)')
plt.xlabel('sample point')
plt.ylabel('value')
plt.legend()

# 显示图像
plt.tight_layout()
plt.show()

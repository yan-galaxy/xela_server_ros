import matplotlib.pyplot as plt
import numpy as np

# 读取数据
def read_data(filename):
    with open(filename, 'r') as file:
        data = [float(line.strip()) for line in file]
    return data

# 绘制数据
def plot_data(slopes, rmse, y_data,Catch,lift_order,stable,start_lift):
    plt.figure(figsize=(10, 6))

    window_offset = 20
    # 创建横坐标，单位为0.01秒
    x_values = np.arange(len(y_data)) * 0.01

    # 绘制原始数据y
    plt.plot(x_values, y_data, label='Original y Data', color='red')

    # 绘制斜率数据，右移window_offset个单位
    slope_x_values = np.arange(window_offset, len(slopes) + window_offset) * 0.01
    plt.plot(slope_x_values, slopes, label='Slope', color='blue')

    # 绘制RMSE数据，右移window_offset个单位
    rmse_x_values = np.arange(window_offset, len(rmse) + window_offset) * 0.01
    plt.plot(rmse_x_values, rmse, label='RMSE', color='green')

    # 添加竖直线，例如在 x = 1.0 的位置
    plt.axvline(x=Catch[0]/100.0, color='black', linestyle='--', label='catch')
    plt.axvline(x=lift_order[0]/100.0, color='orange', linestyle='--', label='lift_order')
    plt.axvline(x=stable[0]/100.0, color='green', linestyle='--', label='start_lift')
    plt.axvline(x=start_lift[0]/100.0, color='green', linestyle='--', label='stable')
    
    plt.axhline(y=1.0, color='orange', linestyle='--', label='Y = 2.0s')
    plt.axhline(y=-1.0, color='orange', linestyle='--', label='Y = 2.0s')
    
    
    # 添加图例
    plt.legend()

    # 添加标题和轴标签
    plt.title('Slope, RMSE, and Original y Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')

    # 显示网格
    plt.grid(True)

    # 显示图表
    plt.show()

# 主程序
if __name__ == '__main__':

    slopes = read_data('../data/x_slope.txt')
    rmse = read_data('../data/x_RMSE.txt')
    y_data = read_data('../data/xela_x.txt')

    diff_force = read_data('../data/diff_force.txt')

    Catch = read_data('../data/Catch.txt')
    lift_order = read_data('../data/lift_order.txt')
    stable = read_data('../data/stable.txt')
    start_lift = read_data('../data/start_lift.txt')
    plot_data(slopes, rmse, y_data,Catch,lift_order,stable,start_lift)
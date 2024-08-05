import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# 定义一个函数来读取数据
def read_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # 跳过文件头部可能存在的非数据行
        # 假设第一行是文件名或注释，从第二行开始读取数据
        return [float(line.strip()) for line in lines[1:]]

# 指定文件路径
file_paths = [
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_x.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_y.txt",
    "/home/galaxy/Desktop/Xela_ws/src/threed_viz/data/xela_z.txt"
]

# 读取数据
datax, datay, dataz = [read_data(path) for path in file_paths]

# 创建一个新的图和子图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 为了绘制三维图，我们需要x, y, z坐标
# 这里我们使用数据索引作为x坐标，不同的y坐标来区分数据组
x = np.arange(len(datax))
y1, y2, y3 = np.full_like(x, 1), np.full_like(x, 2), np.full_like(x, 3)

# 绘制三维散点图，这里我们使用不同的y坐标和颜色来区分不同的数据集
scatter1 = ax.scatter(x, y1, datax, label='Data X')
scatter2 = ax.scatter(x, y2, datay, label='Data Y')
scatter3 = ax.scatter(x, y3, dataz, label='Data Z')

# 设置图例
ax.legend()

# 设置坐标轴标签
ax.set_xlabel('Time')
ax.set_ylabel('Group')
ax.set_zlabel('Value')

# 显示图形
plt.show()
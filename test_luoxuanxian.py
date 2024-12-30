import numpy as np
import matplotlib.pyplot as plt

# 螺旋线参数
theta = np.linspace(0, 4 * np.pi, 100)  # 从0到4π生成100个点
r = theta  # 半径与角度成正比，形成螺旋

# 将极坐标转换为笛卡尔坐标
x = r * np.cos(theta)
y = r * np.sin(theta)

# 计算切线向量
dx = np.gradient(x)  # x的导数
dy = np.gradient(y)  # y的导数

# 计算航向角（以弧度为单位）
heading_angles = np.arctan2(dy, dx)

# 将航向角转换为度
heading_angles_degrees = np.degrees(heading_angles)

# 绘制螺旋线和航向角
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(x, y)
plt.title('Spiral Line Starting from (0, 0)')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.axhline(0, color='black', linewidth=0.5, ls='--')  # x轴
plt.axvline(0, color='black', linewidth=0.5, ls='--')  # y轴
plt.grid()
plt.axis('equal')  # 保持比例

# 绘制航向角
plt.subplot(2, 1, 2)
plt.plot(theta, heading_angles_degrees)
plt.title('Heading Angles Along the Spiral')
plt.xlabel('Theta (radians)')
plt.ylabel('Heading Angle (degrees)')
plt.grid()

plt.tight_layout()
plt.show()

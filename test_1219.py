import numpy as np
import matplotlib.pyplot as plt

# 生成 x 值
x = np.linspace(-5, 5, 400)

# 计算 y 值
y = np.exp(-x)

# 绘制图像
plt.plot(x, y, label=r'$y = e^{-x}$')

# 添加标题和标签
plt.title('Plot of $y = e^{-x}$')
plt.xlabel('x')
plt.ylabel('y')

# 添加网格
plt.grid(True)

# 显示图例
plt.legend()

# 显示图像
plt.show()
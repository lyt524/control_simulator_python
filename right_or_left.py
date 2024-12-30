import numpy as np


def point_direction(A, B, P):
    # 将点转换为 NumPy 数组并添加 z 坐标
    A = np.array(A + (0,))
    B = np.array(B + (0,))
    P = np.array(P + (0,))

    # 计算向量 AB 和 AP
    AB = B - A
    AP = P - A

    # 计算叉积
    cross_product = np.cross(AB, AP)

    # 判断方向
    if cross_product[2] > 0:  # 只需检查 z 分量
        return "左侧"
    elif cross_product[2] < 0:
        return "右侧"
    else:
        return "在直线上"


# 示例用法
A = (1, 1)
B = (4, 4)
P = (2, 3)

result = point_direction(A, B, P)
print(result)  # 输出: "右侧"

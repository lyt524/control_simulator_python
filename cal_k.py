import numpy as np


# 三点画圆法
def calculate_curvature_circle(data, interval=1):
    # interval代表计算曲率时选取点的间隔，例如当需要计算data[i]的曲率时，选取的点为
    # data[i-interval],data[i],data[i+interval]这3个点来计算
    # 通过此操作可以缓解离散点不平滑的情况
    curvature_list = []
    for i in range(len(data)):
        # 初始化头尾的曲率均为0
        if i < interval:
            curvature_list.append(0)
        elif i > len(data) - interval - 1:
            curvature_list.append(0)
        # 获取间隔为interval的3个点（二阶差分需要3个点）
        else:
            start, end = i - interval, i + interval + 1
            x, y = data[start:end:interval, 0], data[start:end:interval, 1]

            # 计算圆心
            x1, x2, x3 = x
            y1, y2, y3 = y
            denominator = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))

            x_center = ((x1 ** 2 + y1 ** 2) * (y2 - y3) + (x2 ** 2 + y2 ** 2) * (y3 - y1) + (x3 ** 2 + y3 ** 2) * (
                        y1 - y2)) / denominator
            y_center = ((x1 ** 2 + y1 ** 2) * (x3 - x2) + (x2 ** 2 + y2 ** 2) * (x1 - x3) + (x3 ** 2 + y3 ** 2) * (
                        x2 - x1)) / denominator

            # 计算半径
            r = np.sqrt((x1 - x_center) ** 2 + (y1 - y_center) ** 2)

            # 计算曲率
            curvature = 1 / r

            # 判断曲率的正负
            vector1 = (x2 - x1, y2 - y1)
            vector2 = (x3 - x2, y3 - y2)
            cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]

            if cross_product > 0:
                curvature *= 1  # 曲率为负
            else:
                curvature *= -1  # 曲率为正

            curvature_list.append(curvature)

    # 将头尾数据的曲率设置为最近可计算点的曲率
    curvature_list[0:interval] = [curvature_list[interval]] * interval
    curvature_list[-interval:] = [curvature_list[-interval - 1]] * interval

    return curvature_list



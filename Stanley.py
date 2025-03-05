import math
import numpy as np
import logging
import os
import time
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from Dynamics_model import Dy_Car
from pid_lon import PID
from Kinematics_model import Ki_Car
from generate_path import Path


def cal_k(x_list, y_list):
    k_list = [0]
    for i in range(1, len(x_list) - 1):
        a_index = i - 1
        b_index = i
        c_index = i + 1
        d_a = math.sqrt(pow(x_list[b_index] - x_list[c_index], 2) + pow(y_list[b_index] - y_list[c_index], 2))
        d_b = math.sqrt(pow(x_list[a_index] - x_list[c_index], 2) + pow(y_list[a_index] - y_list[c_index], 2))
        d_c = math.sqrt(pow(x_list[b_index] - x_list[a_index], 2) + pow(y_list[b_index] - y_list[a_index], 2))
        if d_a < 0.01:
            d_a = 0.01
        if d_c < 0.01:
            d_c = 0.01
        cos = (d_a * d_a + d_c * d_c - d_b * d_b) / (2 * d_a * d_c)
        # 这里可能是精度上有差异，竟然能算出大于1的cos
        if (1 - cos * cos) < 0:
            k_temp = 0.01
        else:
            sin = math.sqrt(1 - cos * cos)
            k_temp = 2 * sin / d_b
        k_list.append(k_temp)
    k_list.append(0)
    return k_list


def Normalize_angle(angle):  # 整定一个弧度到 -pi to pi
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


class Stanley:
    def __init__(self):
        self.stanley_k = 1
        self.index = 0
        self.pre_index = 0
        self.distance = 0
        self.e_list = []

    def set_delta_f(self, model, planning_x_list, planning_y_list, planning_h_list, kd_tree):
        cur_car_x = model.x
        cur_car_y = model.y

        self.distance, self.index = kd_tree.query([cur_car_x, cur_car_y])
        if self.index < self.pre_index:
            self.index = self.pre_index
        else:
            self.pre_index = self.index

        e_phi = Normalize_angle(planning_h_list[self.index] - model.phi)
        alpha = math.atan2(planning_y_list[self.index] - cur_car_y, planning_x_list[self.index] - cur_car_x)
        print("e_phi = ", e_phi)
        e = np.sign(np.sin(Normalize_angle(alpha - model.phi))) * self.distance
        # e = self.distance

        self.e_list.append(e)
        print("e = ", e)
        delta_e = math.atan2(self.stanley_k * e, 4*model.vx)
        print("delta_e = ", delta_e)
        return Normalize_angle(e_phi + delta_e)


if __name__ == '__main__':
    # 螺旋线参数
    theta = np.linspace(0, 4 * np.pi, 100)  # 从0到4π生成100个点
    r = 2*theta  # 半径与角度成正比，形成螺旋

    # 将极坐标转换为笛卡尔坐标
    planning_x_list = r * np.cos(theta)
    planning_y_list = r * np.sin(theta)

    # 计算切线向量
    planing_dx = np.gradient(planning_x_list)  # x的导数
    planing_dy = np.gradient(planning_y_list)  # y的导数

    # 计算航向角（以弧度为单位）
    planning_h_list = np.arctan2(planing_dy, planing_dx)
    # planning_k_list = cal_k(planning_x_list, planning_y_list)

    # FILENAME = 'target_list_2024-10-21_14_49_29.txt'
    # tar_path = Path(FILENAME)
    # tar_path.path_preparation()

    # refer_path = np.zeros((len(tar_path.planning_x_list), 2))
    # refer_path[:, 0] = tar_path.planning_x_list
    # refer_path[:, 1] = tar_path.planning_y_list
    # print(refer_path[:, 0])
    # print("_________________________________")
    # print(refer_path[:, 1])
    # my_KDTree = KDTree(refer_path)

    refer_path = np.zeros((len(planning_x_list), 2))
    refer_path[:, 0] = planning_x_list
    refer_path[:, 1] = planning_y_list
    print(refer_path[:, 0])
    print("_________________________________")
    print(refer_path[:, 1])
    my_KDTree = KDTree(refer_path)

    # ki_car = Ki_Car(-1058.5960008467866, -12017.00799999886, 3.14, 2.9, 1, 0.01)
    ki_car = Ki_Car(0, 0, 0, 2.9, 1, 0.01)
    stanley = Stanley()

    for i in range(5000):
        delta_f = stanley.set_delta_f(ki_car, planning_x_list, planning_y_list, planning_h_list, my_KDTree)
        ki_car.update(delta_f)
        print("index = ", stanley.index)
        print("delta_f = ", delta_f)
        print("_____________________")

    # plot
    print(planning_h_list)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5))

    # plt.plot(ki_car.x_list, stanley.e_list, 'b', linewidth=2.0)
    # plt.plot(refer_path[:, 0], refer_path[:, 1], '.r', linewidth=1.0)
    ax1.plot(ki_car.x_list, stanley.e_list, label='sin(x)', color='blue')
    ax1.set_title('Sine Function')
    ax1.set_xlabel('x')
    ax1.set_ylabel('sin(x)')
    ax1.grid(True)
    ax1.legend()

    ax2.plot(ki_car.x_list, ki_car.y_list, label='cos(x)', color='blue')
    ax2.plot(refer_path[:, 0], refer_path[:, 1], label='cos(x)', color='green')
    ax2.set_title('Cosine Function')
    ax2.set_xlabel('x')
    ax2.set_ylabel('cos(x)')
    ax2.grid(True)
    ax2.legend()

    # 调整布局
    plt.tight_layout()

    # 显示图形
    plt.show()


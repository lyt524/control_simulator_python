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


def Normalize_angle(angle):
    while angle < - math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle


def rad2deg(angle):
    return angle * 180 / math.pi


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


class Node:
    def __init__(self, x, y, phi, kdtree, tar_x_list, tar_y_list):
        self.x = x
        self.y = y
        self.phi = phi
        self.length = 0.5
        self.another_x = self.x + self.length * math.cos(self.phi)
        self.another_y = self.y + self.length * math.sin(self.phi)
        self.visual_x_list = [self.x, self.another_x]
        self.visual_y_list = [self.y, self.another_y]
        self.alpha = 0
        self.kdtree = kdtree
        self.match_i = 0
        self.ed = 0
        self.tar_x_list = tar_x_list
        self.tar_y_list = tar_y_list
        self.ed_with_dir = 0  # with dir
        self.car_vec = np.array([])
        self.x_minus_x_tar = np.array([])
        self.dir = 0

    def find_match_point(self):
        self.ed, self.match_i = self.kdtree.query([self.x, self.y])

    def cal_ed(self):
        self.alpha = math.atan2(self.tar_y_list[self.match_i] - self.y, self.tar_x_list[self.match_i] - self.x)
        self.ed_with_dir = np.sign(math.sin(self.alpha - self.phi)) * self.ed

    def cal_dir(self):
        self.car_vec = np.array([self.another_x - self.x, self.another_y - self.y, 0])
        self.x_minus_x_tar = np.array([self.x - self.tar_x_list[self.match_i], self.y - self.tar_y_list[self.match_i], 0])
        self.dir = np.sign(np.cross(self.car_vec, self.x_minus_x_tar)) * 1


if __name__ == '__main__':
    tar_x_list = []
    tar_y_list = []
    tar_k_list = []
    sequence = np.arange(0, 10.1, 0.1)

    for i in sequence:
        xi = i
        yi = math.sin(xi)
        tar_x_list.append(xi)
        tar_y_list.append(yi)

    tar_k_list = cal_k(tar_x_list, tar_y_list)
    refer_path = np.zeros((len(sequence), 2))
    refer_path[:, 0] = tar_x_list
    refer_path[:, 1] = tar_y_list
    my_kdtree = KDTree(refer_path)

    node1 = Node(1, 0, 0, my_kdtree, tar_x_list, tar_y_list)
    node1.find_match_point()
    node1.cal_ed()
    node1.cal_dir()

    node2 = Node(1, 1, 0, my_kdtree, tar_x_list, tar_y_list)
    node2.find_match_point()
    node2.cal_ed()
    node2.cal_dir()

    node3 = Node(1, 1, 0, my_kdtree, tar_x_list, tar_y_list)
    node3.find_match_point()
    node3.cal_ed()
    node3.cal_dir()

    print("deg node1.alpha = ", rad2deg(node1.alpha))
    print("deg node1.phi = ", rad2deg(node1.phi))
    print("node1.ed = ", node1.ed)
    print("node1.ed_with_dir = ", node1.ed_with_dir)
    print("node1.dir = ", node1.dir)
    print("________")

    print("deg node2.alpha = ", rad2deg(node2.alpha))
    print("deg node2.phi = ", rad2deg(node2.phi))
    print("node2.ed = ", node2.ed)
    print("node2.ed_with_dir = ", node2.ed_with_dir)
    print("node2.dir = ", node2.dir)
    print("________")

    print("deg node3.alpha = ", rad2deg(node3.alpha))
    print("deg node3.phi = ", rad2deg(node3.phi))
    print("node3.ed = ", node3.ed)
    print("node3.ed_with_dir = ", node3.ed_with_dir)
    print("node3.dir = ", node3.dir)
    print("________")

    # print("len(tar_x_list) = ", len(tar_x_list))
    # print("len(tar_k_list) = ", len(tar_k_list))

    # plot
    plt.plot(tar_x_list, tar_y_list, 'g', linewidth=2.0)
    plt.plot(node1.visual_x_list, node1.visual_y_list, 'b', linewidth=2.0)
    plt.plot(node1.x, node1.y, 'b', marker='o')
    plt.plot(tar_x_list[node1.match_i], tar_y_list[node1.match_i], 'b', marker='o')

    plt.plot(node2.visual_x_list, node2.visual_y_list, 'r', linewidth=2.0)
    plt.plot(node2.x, node2.y, 'r', marker='o')
    plt.plot(tar_x_list[node2.match_i], tar_y_list[node2.match_i], 'r', marker='o')

    plt.plot(node3.visual_x_list, node3.visual_y_list, 'r', linewidth=2.0)
    plt.plot(node3.x, node3.y, 'r', marker='o')
    plt.plot(tar_x_list[node3.match_i], tar_y_list[node3.match_i], 'r', marker='o')

    # plt.plot(tar_x_list, tar_k_list, '.r', linewidth=1.0)
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.title('XY')
    plt.grid()
    plt.show()

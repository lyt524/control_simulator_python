# from Kinematics_model import UGV_model
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import math


class UGV_model:
    def __init__(self, x0, y0, theta0, L, v0, T):  # L:wheel base
        self.x = x0  # X
        self.y = y0  # Y
        self.theta = theta0  # heading
        self.l = L  # wheel base
        self.v = v0  # speed
        self.dt = T  # decision time periodic
        self.path_x = []  # Store path for plotting
        self.path_y = []

    def update(self, vt, deltat):
        dx = self.v * np.cos(self.theta)
        dy = self.v * np.sin(self.theta)
        d_theta = self.v * np.tan(deltat) / self.l
        self.x += dx * self.dt
        self.y += dy * self.dt
        self.theta += d_theta * self.dt
        self.path_x.append(self.x)
        self.path_y.append(self.y)


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e_pre = 0
        self.e_list = []
        self.pre_index = 0
        self.dist_list = []
        self.index_list = []
        self.err_list = []
        self.u_list = []

    def set_u(self, e):
        if len(self.e_list) < 10:
            self.e_list.append(e)
        else:
            self.e_list.pop(0)
            self.e_list.append(e)
        p = self.kp * e
        i = self.ki * sum(self.e_list)
        d = self.kd * ((e - self.e_pre) / 0.1)
        u = p + i + d
        if u > np.pi / 6:
            u = np.pi / 6
        if u < - np.pi / 6:
            u = - np.pi / 6
        self.u_list.append(u)
        self.e_pre = e
        print("u = ", u)
        return u

    def get_err(self, car_model, ref_kd_tree, ref_path):
        car_state = np.zeros(2)
        car_x = car_model.x
        car_y = car_model.y
        car_state[0] = car_x
        car_state[1] = car_y
        _, index = ref_kd_tree.query([car_x, car_y])
        self.index_list.append(index)

        # 防止逆行逻辑
        if index < self.pre_index:
            index = self.pre_index
            print("nixing")
        else:
            self.pre_index = index

        # 计算横向距离
        # distance = np.linalg.norm(car_state - ref_path[index])
        distance = math.sqrt((car_state[0] - ref_path[index][0]) ** 2 + (car_state[1] - ref_path[index][1]) ** 2)
        self.dist_list.append(distance)
        print("distance", distance)
        dx, dy = ref_path[index] - car_state
        alpha = math.atan2(dy, dx)
        print("alpha = ", alpha)
        err = (math.sin(alpha - car_model.phi)) * distance
        self.err_list.append(err)
        print("err = ", err)
        return err


if __name__ == '__main__':
    # 初始化道路
    refer_path = np.zeros((1000, 2))
    refer_path[:, 0] = np.linspace(0, 1000, 1000)
    refer_tree = KDTree(refer_path)  # 构建KD树
    # plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=5.0)

    # 实例化对象
    ugv = UGV_model(0, 1.0, 0, 2.0, 2.0, 0.1)
    pid = PID(0.005, 0, 0.01)

    # run!
    for i in range(1000):
        err = pid.get_err(ugv, refer_tree, refer_path)
        u = pid.set_u(err)
        ugv.update(2.0, u)

    # Show the plot
    plt.plot(ugv.path_x, ugv.path_y, 'b', linewidth=1.0)
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.title('XY')
    plt.grid()
    plt.show()
    # print(len(pid.dist_list))

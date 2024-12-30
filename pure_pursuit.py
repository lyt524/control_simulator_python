from Kinematics_model import Ki_Car
from Dynamics_model import Dy_Car
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import sys


class Pure_Pursuit:
    def __init__(self):
        self.pre_index = 0
        self.index = 0
        self.distance = 0
        self.alpha = 0
        self.k = 0.2  # 前视距离系数
        self.lfc = 1  # 前视距离常数
        self.path_index_length = 3
        self.path_start_index = 0  # 前视点窗口起始点
        self.path_end_index = self.path_start_index + self.path_index_length  # 前视点窗口结束点
        self.find_lf_point_flag = 0

    def get_lf_point_alpha(self, car_model, ref_path):
        car_x = car_model.x
        car_y = car_model.y
        car_theta = car_model.phi
        # 选取前视点
        for path_i in range(self.path_start_index, self.path_end_index):
            self.distance = math.sqrt((car_x - ref_path[path_i][0])**2 + (car_y - ref_path[path_i][1])**2)
            # print("self.distance = ", self.distance)
            if self.distance >= self.lfc:
                self.find_lf_point_flag = 1  # found point
                self.index = path_i
                # print("self.index = ", self.index)
                # print("self.path_start_index = ", self.path_start_index)
                self.path_start_index = self.index
                self.path_end_index = self.path_start_index + self.path_index_length
                if self.path_end_index >= ref_path.shape[0]:  # 防越路径终点界
                    self.path_end_index = ref_path.shape[0]
                break

        # 是否选取成功
        if self.find_lf_point_flag == 0:
            sys.exit("can't find lf_point.")
        # 重置
        self.find_lf_point_flag = 0

        # 防逆行逻辑
        if self.index < self.pre_index:
            self.index = self.pre_index
            # print("retrograde")
        else:
            self.pre_index = self.index

        dx = ref_path[self.index][0] - car_x
        dy = ref_path[self.index][1] - car_y
        self.alpha = math.atan2(dy, dx) - car_theta

    def set_delta_f(self, car_model):
        lf = self.k * car_model.vx + self.lfc  # lf是车速的函数 lf = k * vx + lfc
        delta_f = math.atan(car_model.L * 2 * math.sin(self.alpha) / lf)
        return delta_f


if __name__ == '__main__':
    # 生成路径及KD树
    refer_path = np.zeros((100, 2))
    refer_path[:, 0] = np.linspace(0, 50, 100)  # 0 - 50的长度 100个点
    for i in range(refer_path.shape[0]):
        refer_path[i, 1] = math.sin(refer_path[i, 0] / 5.0) * refer_path[i, 0] / 2.0
    refer_tree = KDTree(refer_path)  # KD树在纯跟踪中未用到

    # 实例化车辆，控制算法
    ki_car = Ki_Car(0, 0, 0, 2.9, 2, 0.01)
    dy_car = Dy_Car(1412, -112600, -89500, 2.0, 1.0, 1.9, 2.9, 1536, 0, 0, 0, 0.01, 0, 0, 0)
    pure_pursuit = Pure_Pursuit()

    # run!
    for i in range(5000):
        pure_pursuit.get_lf_point_alpha(ki_car, refer_path)
        ki_car.update(pure_pursuit.set_delta_f(ki_car))

    # plot
    plt.plot(ki_car.x_list, ki_car.y_list, 'b', linewidth=2.0)
    plt.plot(refer_path[:, 0], refer_path[:, 1], '.r', linewidth=1.0)
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.title('XY')
    plt.grid()
    plt.show()


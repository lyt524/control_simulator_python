import numpy as np
import math
import matplotlib.pyplot as plt


def Normalize_angle(angle):  # 整定一个弧度到-pi to pi
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < - math.pi:
        angle += 2 * math.pi
    return angle


class Dy_Car:
    def __init__(self, M, Cf, Cr, vx, a, b, L, Iz, phi_init, d_phi_init, vy_init, T, x0, y0, steer_init):
        self.m = M
        self.cf = Cf
        self.cr = Cr
        self.vx = vx  # 车身坐标系下vx
        self.a = a
        self.b = b
        self.L = L
        self.Iz = Iz
        self.damping_factor = 0.1
        self.phi = phi_init  # 横摆角
        self.d_phi = d_phi_init  # 横摆角速度
        self.vy = vy_init  # 车身坐标系下vy
        self.dt = T
        self.x = x0
        self.y = y0
        self.steer = steer_init
        self.x_list = []
        self.y_list = []
        self.phi_list = []
        self.steer_list = []
        self.v_list = []

    def update(self, delta_f):
        # target_steer = delta_f
        self.steer = delta_f
        self.steer_list.append(self.steer)
        # self.steer += (target_steer - self.steer) * self.damping_factor
        # print("steer = ", self.steer)
        # print("d_d_phi = ", self.d_phi)
        # print("d_phi = ", self.d_phi)
        d_vy = ((self.cf + self.cr)/(self.m * self.vx)) * self.vy + (((self.a * self.cf - self.b * self.cr)/(self.m * self.vx)) - self.vx) * self.d_phi - (self.cf/self.m) * self.steer
        d_d_phi = ((self.a * self.cf - self.b * self.cr)/(self.Iz * self.vx)) * self.vy + ((self.a * self.a * self.cf + self.b * self.b * self.cr)/(self.Iz * self.vx)) * self.d_phi - (self.a * self.cr/self.Iz) * self.steer
        self.d_phi += d_d_phi * self.dt
        self.phi += self.d_phi * self.dt
        self.phi = Normalize_angle(self.phi)
        self.vy += d_vy * self.dt
        beta = math.atan2(self.vy, self.vx)  # 质心侧偏角
        # print("vy = ", self.vy)
        # print("vx = ", self.vx)
        # print("beta = ", beta)
        # print("phi = ", self.phi)
        theta = Normalize_angle(self.phi + beta)
        # theta = Normalize_angle(self.phi)
        # print("theta = ", theta)
        v_ground = self.vx / math.cos(beta)
        # print("v_ground = ", v_ground)
        vx_ground = v_ground * math.cos(theta)
        # print("vx_ground = ", vx_ground)
        vy_ground = v_ground * math.sin(theta)
        # print("vy_ground = ", vy_ground)
        self.x = self.x + vx_ground * self.dt
        # print("x = ", self.x)
        self.y = self.y + vy_ground * self.dt
        # print("y = ", self.y)
        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.phi_list.append(self.phi)
        self.v_list.append(self.vx)

# car = Car(1412, -112600, -89500, 2, 1.0, 1.9, 2.9, 1536, 0, 0, 0, 0.01, 0, 0, 0)


def deg2rad(angle):
    return angle*math.pi/180


def steer_generate(length, num, amplitude, control_list):
    x = np.linspace(0, length, num)  # 从 0 到 2π，生成 100 个点
    for x_i in x:
        y = amplitude * np.sin(0.5*x_i)
        control_list.append(y)
    return x, control_list


# print(deg2rad(10))
# steer_list = []
# total_i, c_list = steer_generate(20, 100, 0.1, steer_list)
# print(c_list)
# for i in c_list:
#     car.steer = i
#     car.update()


# print(car.y_list)
# plt.plot(car.x_list, car.y_list, label='y = sin(x)', color='b')
# plt.show()


# # 绘制正弦曲线
# plt.figure(figsize=(10, 5))
# plt.plot(x, y, label='y = sin(x)', color='b')
# plt.title('Sine Wave')
# plt.xlabel('x (radians)')
# plt.ylabel('y')
# plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
# plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
# plt.grid()
# plt.legend()
# plt.show()

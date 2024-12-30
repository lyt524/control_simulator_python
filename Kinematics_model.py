import numpy as np
import matplotlib
import matplotlib.pyplot as plt
# %matplotlib inline

# set up matplotlib
# is_ipython = 'inline' in matplotlib.get_backend()
# if is_ipython:
#     from IPython import display

# plt.ion()
# plt.figure(figsize=(18, 3))


class Ki_Car:
    def __init__(self, x0, y0, phi0, L, v0, T):  # L:wheel base
        self.x = x0
        self.y = y0
        self.phi = phi0
        self.L = L
        self.vx = v0
        self.dt = T
        self.x_list = []
        self.y_list = []
        self.phi_list = []
        self.v_list = []
        self.delta_f_list = []

    def update(self, delta_f):
        dx = self.vx * np.cos(self.phi)
        dy = self.vx * np.sin(self.phi)
        d_theta = self.vx * np.tan(delta_f) / self.L
        self.x += dx * self.dt
        self.y += dy * self.dt
        self.phi += d_theta * self.dt

        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.phi_list.append(self.phi)
        self.v_list.append(self.vx)
        self.delta_f_list.append(delta_f)

# refer_path = np.zeros((100, 2))
# refer_path[:, 0] = np.linspace(0, 18, 100)
#
# plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=5.0)
# ugv = UGV_model(0, 0, 0, 2.86, 2.0, 0.01)
# for i in range(1000):
#     ugv.update(2.0, np.cos(i / 5.0))
#     ugv.plot_duration()
#
#
# plt.ioff()  # Disable interactive mode
# plt.show()  # Show the final plot

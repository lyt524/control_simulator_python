import numpy as np
import math
import matplotlib.pyplot as plt


class Path:
    def __init__(self, file_name):
        self.file_name = file_name
        self.planning_x_list = []
        self.planning_y_list = []
        self.planning_h_list = []
        self.planning_v_list = []

    def path_preparation(self):
        data = np.loadtxt(self.file_name, delimiter=' ')
        self.planning_x_list = data[:, 0]
        self.planning_y_list = data[:, 1]
        self.planning_h_list = data[:, 2]
        self.planning_v_list = data[:, 3]





# cx = np.arange(0, 50, 1)
# cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
# print("len(cx) = ", len(cx))
#
# refer_path = np.zeros((100, 2))
# refer_path[:, 0] = np.linspace(0, 50, 100)
# for i in range(refer_path.shape[0]):
#     refer_path[i, 1] = math.sin(refer_path[i, 0] / 5.0) * refer_path[i, 0] / 2.0
#
#
# # Show the plot
# plt.plot(refer_path[:, 0], refer_path[:, 1], 'b', linewidth=1.0)
# plt.xlabel('X Axis')
# plt.ylabel('Y Axis')
# plt.title('XY')
# plt.grid()
# plt.show()


import math
import numpy as np
import matplotlib.pyplot as plt

p1 = (0, 0)
p2 = (4, 0)
p1_p2 = (p2[0] - p1[0], p2[1] - p2[1])
p1p2_length = math.sqrt((p1[0]-p2[0]) ** 2 + (p1[1]-p2[1]) ** 2)
p1_p2_dir = (p1_p2[0] / p1p2_length, p1_p2[1] / p1p2_length)


range_list = np.linspace(0, 2 * np.pi, 100)
p0_length = 2
length_list = []
for theta in range_list:
    p0 = (p0_length * math.cos(theta), p0_length * math.sin(theta))
    p1_p0 = (p0[0] - p1[0], p0[1] - p2[1])
    # minus_result = p1_p0[0] * p1_p2[1] - p1_p0[1] * p1_p2[0]
    minus_result = p1_p2[0] * p1_p0[1] - p1_p2[1] * p1_p0[0]
    minus_result_dir = p1_p2_dir[0] * p1_p0[1] - p1_p2_dir[1] * p1_p0[0]
    tri_result = p0_length * p1p2_length * math.sin(theta)
    p0_2_p1p2 = abs(minus_result_dir)
    length_list.append(p0_2_p1p2)
    if abs(minus_result - tri_result) > 0.001:
        print("???")
    print("theta(deg) = ", 180 * theta / math.pi)
    print("minus_result = ", minus_result)
    print("tri_result = ", tri_result)
    print("length = ", p0_2_p1p2)
    print("________________________________")

# draw
plt.plot(range_list, length_list, label='result')
plt.show()











import numpy as np
import scipy as sp
import scipy.sparse as sparse
import osqp
import warnings
import time
import matplotlib.pyplot as plt

Qu = 2.0 * sparse.eye(1)
# A = np.array([2.1])
# B = Qu * A
pref = 7.0  # 参考位置
vref = 0.0  # 参考速度
xref = np.array([pref, vref])  # reference state 参考状态量
Np = 25
Xref = np.kron(np.ones((Np + 1, 1)), xref)  # 26行1列 的 全1矩阵与 xref 进行kron计算
Ts = 0.2
len_sim = 40
nsim = int(len_sim/Ts)
tsim = np.arange(0, nsim)*Ts
print("tsim.shape = ", tsim.shape)

print("xref = ", xref)
print("Xref = ", Xref)


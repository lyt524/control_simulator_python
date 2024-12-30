from math import log, ceil

max_iter = 1e+2
R = max_iter
eta = 4
s_max = int(log(R) / log(eta))
B = (s_max + 1) * R
print("最大资源量 R = ", R)
print("确定丢弃配置比例值 eta = ", eta)
print("s_max = ", s_max)
print("B = ", B)

for s in reversed(range(s_max + 1)):
    """""""""""""""""""""""""""""""""
    -----     Initialize n,r     ----
    """""""""""""""""""""""""""""""""
    n = int(ceil(B / max_iter / (s + 1) * eta ** s))
    n_keep = n
    r = max_iter * eta ** (-s)
    print(f"  生成n = {n} 组随机参数配置")
    print(f"  r = {r} 用来确定参数配置的迭代次数 r_i")
    for i in range(s + 1):
        r_i = r * eta ** i
        print(f"    这n = {n_keep} 组参数配置中的每一组都要进行 r_i = {r_i} 次噪声迭代")
        for j in range(n_keep):
            """""""""""""""""""""""""""""""""""
            -----    eval with mutation   -----
            """""""""""""""""""""""""""""""""""
            print(f"        n = {n_keep} 组配置中的第 j = {j} 组配置进行r_i = {r_i} 次经过噪声扰动后的新配置重新计算损失的过程")  # 一组配置

        n_keep = int(n * eta ** (-i - 1))
        print(f"    保留了前n_keep = {n_keep} 组参数配置")


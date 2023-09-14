import math
import numpy as np
import control
import control.matlab
import csv

from walking.preview_control import *

if __name__ == '__main__':
    pc = preview_control(0.01, 1.0, 0.27)

    # 输出离散时间状态空间模型的矩阵
    # print("\sys_d:", pc.sys_d)
    # print("\nA_d:", pc.A_d)
    # print("\nB_d:", pc.B_d)
    # print("\nC_d:", pc.C_d)
    #

    foot_step = [
        [0, 0, 0],
        [0.34, 0, 0.06+0.00],
        [0.68, 0.05, -0.06+0.02],
        [1.02, 0.10, 0.06+0.04],
        [1.36, 0.15, -0.06+0.06],
        [1.7, 0.20, 0.06+0.08],
        [2.04, 0.25, 0.0+0.10],
        [2.72, 0.25, 0.0+0.10],
        [100, 0.25, 0.0+0.10]
    ]
    x, y = np.matrix([[0.0], [0.0], [0.0]]), np.matrix([[0.0], [0.0], [0.0]])
    with open('result.csv', mode='w') as f:
        f.write('')
    t = 0
    while True:
        if len(foot_step) <= 2:
            break
        cog, x, y = pc.set_param(t, x, y, foot_step)
        with open('result.csv', mode='a') as f:
            writer = csv.writer(f)
            for i in cog:
                writer.writerow(i.tolist()[0])
            f.write('\n')
        del foot_step[0]
        t = foot_step[0][0]
#  print(pc.set_param([0,0], [0,0], [0,0], foot_step))

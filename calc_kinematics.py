from casadi import *
from numpy import array
from modern_robotics.core import *

L0 = SX.sym('L0')
L1 = SX.sym('L1')
L2 = SX.sym('L2')
L3 = SX.sym('L3')

t1 = SX.sym('t1')
t2 = SX.sym('t2')
t3 = SX.sym('t3')
t4 = SX.sym('t4')
t5 = SX.sym('t5')

dt1 = SX.sym('dt1')
dt2 = SX.sym('dt2')
dt3 = SX.sym('dt3')
dt4 = SX.sym('dt4')
dt5 = SX.sym('dt5')

ddt1 = SX.sym('ddt1')
ddt2 = SX.sym('ddt2')
ddt3 = SX.sym('ddt3')
ddt4 = SX.sym('ddt4')
ddt5 = SX.sym('ddt5')

M = np.array([[ 1, 0, 0, 0],
              [ 0, 1, 0, L0],
              [ 0, 0, 1, -(L1 + L2 + L3)],
              [ 0, 0, 0, 1]])
# print(M)

w1 = np.array([1, 0, 0])
v1 = np.array([0, 0, -L0]).T

w2 = np.array([0, 0, 1])
v2 = np.array([L0, 0, 0]).T

w3 = np.array([0, 1, 0])
v3 = np.array([0, 0, 0]).T

w4 = np.array([0, 1, 0])
v4 = np.array([L1, 0, 0]).T

w5 = np.array([0, 1, 0])
v5 = np.array([L1+L2, 0, 0]).T

Slist = np.array([
    [1, 0, 0, 0, 0, -L0],
    [0, 0, 1, L0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, L1, 0, 0],
    [0, 1, 0, L1+L2, 0, 0]
]).T

# print(Slist)

T = np.array(M)

def calcT(w1, v1, t1):
    omgmat1 = VecToso3(w1)
    R1 = SX.eye(3) + sin(t1) * omgmat1 + (1-cos(t1)) * np.dot(omgmat1, omgmat1)
    p1 = (SX.eye(3) * t1 + (1 - cos(t1)) * omgmat1 + (t1 - sin(t1)) * np.dot(omgmat1, omgmat1)) @ v1

    T_1 = horzcat(R1, p1)
    T_2 = np.array([[0, 0, 0, 1]])
    T1 = vertcat(T_1, T_2)
    return T1

# ---------------------------------------
T1 = calcT(w1, v1, t1)
T2 = calcT(w2, v2, t2)
T3 = calcT(w3, v3, t3)
T4 = calcT(w4, v4, t4)
T5 = calcT(w5, v5, t5)

T = T1 @ T2 @ T3 @ T4 @ T5 @ M

thetalist = np.array([t1, t2, t3, t4, t5])
dthetalist = np.array([dt1, dt2, dt3, dt4, dt5])
ddthetalist = np.array([ddt1, ddt2, ddt3, ddt4, ddt5])

M_5_6 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -L3], [0, 0, 0, 1]])
M_4_5 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -L2], [0, 0, 0, 1]])
M_3_4 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -L1], [0, 0, 0, 1]])
M_2_3 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
M_1_2 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
M_0_1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

m1 = SX.sym('m1')
Ixx1 = SX.sym('Ixx1')
Iyy1 = SX.sym('Iyy1')
Izz1 = SX.sym('Izz1')
Ixy1 = SX.sym('Ixy1')
Ixz1 = SX.sym('Ixz1')
Iyz1 = SX.sym('Iyz1')

m2 = SX.sym('m2')
Ixx2 = SX.sym('Ixx2')
Iyy2 = SX.sym('Iyy2')
Izz2 = SX.sym('Izz2')
Ixy2 = SX.sym('Ixy2')
Ixz2 = SX.sym('Ixz2')
Iyz2 = SX.sym('Iyz2')

m3 = SX.sym('m3')
Ixx3 = SX.sym('Ixx3')
Iyy3 = SX.sym('Iyy3')
Izz3 = SX.sym('Izz3')
Ixy3 = SX.sym('Ixy3')
Ixz3 = SX.sym('Ixz3')
Iyz3 = SX.sym('Iyz3')

m4 = SX.sym('m4')
Ixx4 = SX.sym('Ixx4')
Iyy4 = SX.sym('Iyy4')
Izz4 = SX.sym('Izz4')
Ixy4 = SX.sym('Ixy4')
Ixz4 = SX.sym('Ixz4')
Iyz4 = SX.sym('Iyz4')

m5 = SX.sym('m5')
Ixx5 = SX.sym('Ixx5')
Iyy5 = SX.sym('Iyy5')
Izz5 = SX.sym('Izz5')
Ixy5 = SX.sym('Ixy5')
Ixz5 = SX.sym('Ixz5')
Iyz5 = SX.sym('Iyz5')

G1 = np.array([
    [Ixx1, Ixy1, Ixz1, 0, 0, 0],
    [Ixy1, Iyy1, Iyz1, 0, 0, 0],
    [Ixz1, Iyz1, Izz1, 0, 0, 0],
    [0, 0, 0, m1, 0, 0],
    [0, 0, 0, 0, m1, 0],
    [0, 0, 0, 0, 0, m1],
])

G2 = np.array([
    [Ixx2, Ixy2, Ixz2, 0, 0, 0],
    [Ixy2, Iyy2, Iyz2, 0, 0, 0],
    [Ixz2, Iyz2, Izz2, 0, 0, 0],
    [0, 0, 0, m2, 0, 0],
    [0, 0, 0, 0, m2, 0],
    [0, 0, 0, 0, 0, m2],
])

G3 = np.array([
    [Ixx3, Ixy3, Ixz3, 0, 0, 0],
    [Ixy3, Iyy3, Iyz3, 0, 0, 0],
    [Ixz3, Iyz3, Izz3, 0, 0, 0],
    [0, 0, 0, m3, 0, 0],
    [0, 0, 0, 0, m3, 0],
    [0, 0, 0, 0, 0, m3],
])

G4 = np.array([
    [Ixx4, Ixy4, Ixz4, 0, 0, 0],
    [Ixy4, Iyy4, Iyz4, 0, 0, 0],
    [Ixz4, Iyz4, Izz4, 0, 0, 0],
    [0, 0, 0, m4, 0, 0],
    [0, 0, 0, 0, m4, 0],
    [0, 0, 0, 0, 0, m4],
])

G5 = np.array([
    [Ixx5, Ixy5, Ixz5, 0, 0, 0],
    [Ixy5, Iyy5, Iyz5, 0, 0, 0],
    [Ixz5, Iyz5, Izz5, 0, 0, 0],
    [0, 0, 0, m5, 0, 0],
    [0, 0, 0, 0, m5, 0],
    [0, 0, 0, 0, 0, m5],
])

g = SX.sym('g')
g_ = np.array([0, 0, -g])

#----------------------------------------------------------------
#----------------------------------------------------------------
#----------------------------------------------------------------

n = len(thetalist)
# print(n)
Mi = np.eye(4)
Ai = SX.zeros(6, 6)
AdTi = [[None]] * (n + 1)
Vi = SX.zeros(6, n + 1) #np.zeros((6, n + 1))
Vdi = SX.zeros(6, n + 1)
Vdi[:, 0] = np.r_[[0, 0, 0], -np.array(g_)]

Mlist = np.array([M_0_1, M_1_2, M_2_3, M_3_4, M_4_5, M_5_6])
#
AdTi[n] = Adjoint(TransInv(Mlist[n]))

Fi = np.array([0, 0, 0, 0, 0, 0])

Glist = np.array([G1, G2, G3, G4, G5])
# print(Slist)
#
taulist = SX.zeros(n)
for i in range(n):
    Mi = Mi @ Mlist[i]
    ad_temp = Adjoint(TransInv(Mi))
    s_temp = np.array(Slist)[:, i]
    # print(ad_temp)
    # print(s_temp)
    Ai[:, i] = ad_temp @ s_temp #np.dot(Adjoint(TransInv(Mi)), np.array(Slist)[:, i])
    # print(Ai[:, i])
    temp = Ai[:, i]# * -thetalist[i]
    # print(temp)
    # print(temp.shape)
    Mlisti_inv = TransInv(Mlist[i])
    w = np.array([temp[0, 0], temp[1, 0], temp[2, 0]])
    v = np.array([temp[3, 0], temp[4, 0], temp[5, 0]])
    # print(w)
    # print(v)
    T_temp = calcT(w, v, thetalist[i])
    R, p = TransToRp(T_temp)
    # print(T_temp)
    AdTi[i] = vertcat(horzcat(R, np.zeros((3, 3))), horzcat(VecToso3(p) @ R, R))
    # print(AdTi[i])
    # AdTi[i] = Adjoint(np.dot(MatrixExp6(VecTose3(Ai[:, i] * -thetalist[i])), TransInv(Mlist[i])))
    Vi[:, i + 1] = AdTi[i] @ Vi[:,i] + Ai[:, i] * dthetalist[i]
    Vdi[:, i + 1] = AdTi[i] @ Vdi[:, i] + Ai[:, i] * ddthetalist[i] + ad(Vi[:, i + 1]) @ Ai[:, i] * dthetalist[i]

print(f"Vi: {Vi}")
print(f"Vdi: {Vdi}")

for i in range (n - 1, -1, -1):
    Fi_temp = AdTi[i + 1].T @ Fi + Glist[i] @ Vdi[:, i + 1] - ad(Vi[:, i + 1]).T @ (Glist[i] @ Vi[:, i + 1])
    # Fi = np.dot(np.array(AdTi[i + 1]).T, Fi) + np.dot(np.array(Glist[i]), Vdi[:, i + 1]) - np.dot(np.array(ad(Vi[:, i + 1])).T, np.dot(np.array(Glist[i]), Vi[:, i + 1]))
    taulist[i] = Fi_temp.T @ Ai[:, i] #np.dot(np.array(Fi).T, Ai[:, i])
# return taulist
print(taulist)

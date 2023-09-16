from casadi import *
import numpy as np

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

m1 = SX.sym('m1')
cx1 = SX.sym('cx1')
cy1 = SX.sym('cy1')
cz1 = SX.sym('cz1')
Ixx1 = SX.sym('Ixx1')
Iyy1 = SX.sym('Iyy1')
Izz1 = SX.sym('Izz1')
Ixy1 = SX.sym('Ixy1')
Ixz1 = SX.sym('Ixz1')
Iyz1 = SX.sym('Iyz1')

m2 = SX.sym('m2')
cx2 = SX.sym('cx2')
cy2 = SX.sym('cy2')
cz2 = SX.sym('cz2')
Ixx2 = SX.sym('Ixx2')
Iyy2 = SX.sym('Iyy2')
Izz2 = SX.sym('Izz2')
Ixy2 = SX.sym('Ixy2')
Ixz2 = SX.sym('Ixz2')
Iyz2 = SX.sym('Iyz2')

m3 = SX.sym('m3')
cx3 = SX.sym('cx3')
cy3 = SX.sym('cy3')
cz3 = SX.sym('cz3')
Ixx3 = SX.sym('Ixx3')
Iyy3 = SX.sym('Iyy3')
Izz3 = SX.sym('Izz3')
Ixy3 = SX.sym('Ixy3')
Ixz3 = SX.sym('Ixz3')
Iyz3 = SX.sym('Iyz3')

m4 = SX.sym('m4')
cx4 = SX.sym('cx4')
cy4 = SX.sym('cy4')
cz4 = SX.sym('cz4')
Ixx4 = SX.sym('Ixx4')
Iyy4 = SX.sym('Iyy4')
Izz4 = SX.sym('Izz4')
Ixy4 = SX.sym('Ixy4')
Ixz4 = SX.sym('Ixz4')
Iyz4 = SX.sym('Iyz4')

m5 = SX.sym('m5')
cx5 = SX.sym('cx5')
cy5 = SX.sym('cy5')
cz5 = SX.sym('cz5')
Ixx5 = SX.sym('Ixx5')
Iyy5 = SX.sym('Iyy5')
Izz5 = SX.sym('Izz5')
Ixy5 = SX.sym('Ixy5')
Ixz5 = SX.sym('Ixz5')
Iyz5 = SX.sym('Iyz5')

g = SX.sym('g')

ax1 = SX.sym('ax1')
ax2 = SX.sym('ax2')
ax3 = SX.sym('ax3')
ax4 = SX.sym('ax4')
ax5 = SX.sym('ax5')

ay1 = SX.sym('ay1')
ay2 = SX.sym('ay2')
ay3 = SX.sym('ay3')
ay4 = SX.sym('ay4')
ay5 = SX.sym('ay5')

az1 = SX.sym('az1')
az2 = SX.sym('az2')
az3 = SX.sym('az3')
az4 = SX.sym('az4')
az5 = SX.sym('az5')

def S_mat(omg):
    return np.array([[0, -omg[2], omg[1]], [omg[2], 0, -omg[0]], [-omg[1], omg[0], 0]])

def K_mat(omg):
    return np.array([
        [omg[0], omg[1], omg[2], 0, 0, 0],
        [0, omg[0], 0, omg[1], omg[2], 0], 
        [0, 0, omg[0], 0, omg[1], omg[2]]])

def matrix_exp3(omg, theta):
    omgmat = S_mat(omg)
    R = numpy.eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * (omgmat @ omgmat)
    return R

P1 = np.array([[m1, m1 * cx1, m1 * cy1, m1 * cz1, Ixx1, Ixy1, Ixz1, Iyy1, Iyz1, Izz1]]).T
P2 = np.array([[m2, m2 * cx2, m2 * cy2, m2 * cz2, Ixx2, Ixy2, Ixz2, Iyy2, Iyz2, Izz2]]).T
P3 = np.array([[m3, m3 * cx3, m3 * cy3, m3 * cz3, Ixx3, Ixy3, Ixz3, Iyy3, Iyz3, Izz3]]).T
P4 = np.array([[m4, m4 * cx4, m4 * cy4, m4 * cz4, Ixx4, Ixy4, Ixz4, Iyy4, Iyz4, Izz4]]).T
P5 = np.array([[m5, m5 * cx5, m5 * cy5, m5 * cz5, Ixx5, Ixy5, Ixz5, Iyy5, Iyz5, Izz5]]).T

w1 = np.array([1, 0, 0])
p1 = np.array([0, -L0, 0])
a1 = np.array([ax1, ay1, az1])

w2 = np.array([0, 0, 1])
p2 = np.array([0, 0, 0])
a2 = np.array([ax2, ay2, az2])

w3 = np.array([0, 1, 0])
p3 = np.array([0, 0, 0])
a3 = np.array([ax3, ay3, az3])

w4 = np.array([0, 1, 0])
p4 = np.array([0, 0, -L1])
a4 = np.array([ax4, ay4, az4])

w5 = np.array([0, 1, 0])
p5 = np.array([0, 0, -L2])
a5 = np.array([ax5, ay5, az5])

R1 = matrix_exp3(w1, t1)
Sp1 = S_mat(p1)
T1 = vertcat(
    horzcat(R1, np.zeros((3, 3))),
    horzcat(Sp1 @ R1, R1)
)

R2 = matrix_exp3(w2, t2);
Sp2 = S_mat(p2);
T2 = vertcat(
    horzcat(R2, np.zeros((3, 3))),
    horzcat(Sp2 @ R2, R2)
)

R3 = matrix_exp3(w3, t3);
Sp3 = S_mat(p3);
T3 = vertcat(
    horzcat(R3, np.zeros((3, 3))),
    horzcat(Sp3 @ R3, R3)
)

R4 = matrix_exp3(w4, t4);
Sp4 = S_mat(p4);
T4 = vertcat(
    horzcat(R4, np.zeros((3, 3))),
    horzcat(Sp4 @ R4, R4)
)

R5 = matrix_exp3(w5, t5);
Sp5 = S_mat(p5);
T5 = vertcat(
    horzcat(R5, np.zeros((3, 3))),
    horzcat(Sp5 @ R5, R5)
)

T11 = np.eye(6);
T12 = T11 @ T1;
T13 = T12 @ T2;
T14 = T13 @ T3;
T15 = T14 @ T4;

T22 = np.eye(6);
T23 = T22 @ T2;
T24 = T23 @ T3;
T25 = T24 @ T4;

T33 = np.eye(6);
T34 = T33 @ T3;
T35 = T34 @ T4;

T44 = np.eye(6);
T45 = T44 @ T4;

T55 = np.eye(6);

g_ = np.array([0, 0, -g])

A1_1 = horzcat(horzcat(a1 - g_, S_mat(w1 * dt1) + S_mat(w1 * t1) @ S_mat(w1 * t1)), np.zeros((3, 6)))
A1_2 = horzcat(horzcat(np.zeros((3, 1)), S_mat(g_- a1)), K_mat(w1 * dt1) + S_mat(w1 * t1) @ K_mat(w1 * t1))
A1 = vertcat(A1_1, A1_2)

A2_1 = horzcat(horzcat(a2 - g_, S_mat(w2 * dt2) + S_mat(w2 * t2) @ S_mat(w2 * t2)), np.zeros((3, 6)))
A2_2 = horzcat(horzcat(np.zeros((3, 1)), S_mat(g_- a2)), K_mat(w2 * dt2) + S_mat(w2 * t2) @ K_mat(w2 * t2))
A2 = vertcat(A2_1, A2_2)

A3_1 = horzcat(horzcat(a3 - g_, S_mat(w3 * dt3) + S_mat(w3 * t3) @ S_mat(w3 * t3)), np.zeros((3, 6)))
A3_2 = horzcat(horzcat(np.zeros((3, 1)), S_mat(g_- a3)), K_mat(w3 * dt3) + S_mat(w3 * t3) @ K_mat(w3 * t3))
A3 = vertcat(A3_1, A3_2)

A4_1 = horzcat(horzcat(a4 - g_, S_mat(w4 * dt4) + S_mat(w4 * t4) @ S_mat(w4 * t4)), np.zeros((3, 6)))
A4_2 = horzcat(horzcat(np.zeros((3, 1)), S_mat(g_- a4)), K_mat(w4 * dt4) + S_mat(w4 * t4) @ K_mat(w4 * t4))
A4 = vertcat(A4_1, A4_2)

A5_1 = horzcat(horzcat(a5 - g_, S_mat(w5 * dt5) + S_mat(w5 * t5) @ S_mat(w5 * t5)), np.zeros((3, 6)))
A5_2 = horzcat(horzcat(np.zeros((3, 1)), S_mat(g_- a5)), K_mat(w5 * dt5) + S_mat(w5 * t5) @ K_mat(w5 * t5))
A5 = vertcat(A5_1, A5_2)

U11 = T11 @ A1
U12 = T12 @ A2
U13 = T13 @ A3
U14 = T14 @ A4
U15 = T15 @ A5

U22 = T22 @ A2
U23 = T23 @ A3
U24 = T24 @ A4
U25 = T25 @ A5

U33 = T33 @ A3
U34 = T34 @ A4
U35 = T35 @ A5

U44 = T44 @ A4
U45 = T45 @ A5

U55 = T55 @ A5

Uz = np.zeros((6, 10));

U_1 = horzcat(U11, U12, U13, U14, U15)
U_2 = horzcat(Uz, U22, U23, U24, U25)
U_3 = horzcat(Uz, Uz, U33, U34, U35)
U_4 = horzcat(Uz, Uz, Uz, U44, U45)
U_5 = horzcat(Uz, Uz, Uz, Uz, U55)

U = vertcat(U_1, U_2, U_3, U_4, U_5)

phi = vertcat(P1, P2, P3, P4, P5)

Y1 = np.array([[0, 0, 0, 1, 0, 0]])
Y2 = np.array([[0, 0, 0, 0, 0, 1]])
Y3 = np.array([[0, 0, 0, 0, 1, 0]])
Y4 = np.array([[0, 0, 0, 0, 1, 0]])
Y5 = np.array([[0, 0, 0, 0, 1, 0]])

K11 = Y1 @ U11
K12 = Y2 @ U11
K13 = Y3 @ U11
K14 = Y4 @ U11
K15 = Y5 @ U11

K22 = Y2 @ U11
K23 = Y3 @ U11
K24 = Y4 @ U11
K25 = Y5 @ U11

K33 = Y3 @ U11
K34 = Y4 @ U11
K35 = Y5 @ U11

K44 = Y4 @ U11
K45 = Y5 @ U11

K55 = Y5 @ U11

Kz = np.zeros((1, 10))

K = vertcat(
    horzcat(K11, K12, K13, K14, K15),
    horzcat(Kz, K22, K23, K24, K25),
    horzcat(Kz, Kz, K33, K34, K35),
    horzcat(Kz, Kz, Kz, K44, K45),
    horzcat(Kz, Kz, Kz, Kz, K55)
)

w_all = U @ phi;
tau = K @ phi
print(tau.shape)


from casadi import *
import numpy as np

def S_mat(omg):
    return np.array([[0, -omg[2], omg[1]], [omg[2], 0, -omg[0]], [-omg[1], omg[0], 0]])

def K_mat(omg):
    return np.array([
        [omg[0], omg[1], omg[2], 0, 0, 0],
        [0, omg[0], 0, omg[1], omg[2], 0], 
        [0, 0, omg[0], 0, omg[1], omg[2]]])

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

P1 = np.array([Ixx1, Ixy1, Ixz1, Iyy1, Iyz1, Izz1, m1 * cx1, m1 * cy1, m1 * cz1, m1])
print(P1)




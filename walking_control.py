#!/usr/bin/env python3
import math
import time
import csv
import sys
import numpy as np
import pybullet_data
import pybullet as p
from random import random
import matplotlib.pyplot as plt

#
sys.path.append('./walking')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *

if __name__ == '__main__':
    TIME_STEP = 0.001
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)
    # name = "MOS"
    name = "PAI"

    planeId = p.loadURDF("plane.urdf", [0, 0, 0])
    if name == "MOS":
        RobotId = p.loadURDF("urdf/urdf/thmos_mix.urdf", [0, 0, 0.43])  # 0.43
    elif name == "PAI":
        RobotId = p.loadURDF("urdf/PAI-urdf/urdf/PAI-urdf.urdf", [0, 0, 0.36])  # 0.43

    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=60, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(RobotId)):
        index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id
        joint_info = p.getJointInfo(RobotId, id)
        # joint_name = joint_info[1]#.decode("utf-8")
        # parent_link_index = joint_info[16]
        # joint_axis = joint_info[13]
        print(f"Joint Name: {id} / {joint_info}")

    """
    p.stepSimulation()
    # step=[-0.1757420939157431, -1.436788534533084e-17, -0.19524765891517334, 0.6839480603251209, -0.4887004014099475, 0.09663387327095674, -3.571150834983083e-18, 0.21918723670487872, -0.7427095274796556, 0.5235222907747769]
    # step=[-0.1292951490286509, -3.166841537489331e-17, -0.48682916878665894, 1.0999027507731163, -0.6130735819864571, 0.12852635603101242, -1.5833934186986698e-17, 0.48698478280096225, -1.1002265273380716, 0.6132417445371093]
    # step=[-0.0997266711501729, -3.8236485680305286e-17, -0.7531806746489149, 1.3252452348281154, -0.5720645601792006, 0.09961063315891033, 0.0, 0.7531964685380437, -1.325274732430613, 0.5720782638925692]
    step=[-0.22380566786759776, 0.0, -0.7008467940077393, 1.2777251605488935, -0.5768783665411543, -0.027611797820207607, -4.677204825014602e-18, 0.7346570128512445, -1.3422314863464142, 0.6075744734951696]
    while 1:
        t=input()
        if (t=='q'): 
            p.disconnect()
            break
            # sleep(0.1)  # Time in seconds.
        else:
            # for i in range(10):
            p.setJointMotorControl2(RobotId, 5 ,p.POSITION_CONTROL, step[5])#math.pi/2)
            p.stepSimulation()
    """
    z = 0.30
    if name == "MOS":
        left_foot0 = p.getLinkState(RobotId, index['L_leg_6_link'])[0]
        right_foot0 = p.getLinkState(RobotId, index['R_leg_6_link'])[0]
        z = 0.30
    elif name == "PAI":
        left_foot0 = p.getLinkState(RobotId, index['left-ankle'])[0]
        right_foot0 = p.getLinkState(RobotId, index['right-ankle'])[0]
        z = 0.40
    print(f'state:\nleft:{left_foot0}\nright:{right_foot0}')

    # """
    joint_angles = []
    for id in range(p.getNumJoints(RobotId)):
        if p.getJointInfo(RobotId, id)[3] > -1:
            joint_angles += [0,]

    left_foot = left_foot0
    right_foot = right_foot0
    # left_foot  = [ left_foot0[0]-0.0,  left_foot0[1]+0.01,  left_foot0[2]-0.04]
    # right_foot = [right_foot0[0]-0.0, right_foot0[1]-0.01, right_foot0[2]-0.04]
    print("current robot is {}, z is ".format(name, z))
    pc = preview_control(0.01, 1.0, z)
    walk = walking(RobotId, left_foot, right_foot, joint_angles, pc, name)

    index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(RobotId)):
        index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

    # goal position (x, y) theta
    t1 = time.time()
    print("\n\n ==n is null==\n")
    foot_step = walk.setGoalPos([1.5, 0.0, 0])
    print(f'set goal time:{time.time()-t1}')

    print("foot_step:", foot_step)

    # init position
    # init_pos = [-0.09,0.0,-0.49,1.11,-0.61,0.09,0.0,0.49,-1.11,0.62]
    # init_pos = [0.0,0.0,-0.29,1.11,-0.61,0.0,0.0,0.29,-1.11,0.62]
    init_pos, _, _, _, _ = walk.getNextPos()
    for i in range(200):
        p.stepSimulation()
    for t in range(200):
        if t % 20 == 0:
            print("time is "+str(t))
        current_pos = [angle*t/200 for angle in init_pos]
        for i in range(len(init_pos)):
            p.setJointMotorControl2(RobotId, i, p.POSITION_CONTROL, current_pos[i], force=2)
        p.stepSimulation()
    joint_angles = init_pos
    # for i in range(10000):
    #   p.stepSimulation()
    j = 0
    step = 0
    joint_angles_recorder = np.array([np.nan for i in range(10)])
    while p.isConnected():
        # contact_points = p.getContactPoints(RobotId, -1)
        # if len(contact_points)<=0:
        #   # print("robot isn't in contact with the ground.")
        #   p.stepSimulation()
        #   continue
        j += 1
        if j >= 10:
            joint_angles, lf, rf, xp, n = walk.getNextPos()
            joint_angles_recorder = np.row_stack((joint_angles_recorder, np.array(joint_angles)))
            print(f'\n-> {step}_joint_angles is:{joint_angles}/{lf}/{rf}/{n}\n')
            j = 0
            if n == 0:
                if (len(foot_step) <= 5):
                    x_goal, y_goal, th = random()-0.5, random()-0.5, random()-0.5
                    break
                    foot_step = walk.setGoalPos([x_goal, y_goal, th])
                else:
                    print("\n\n ==n is null==\n")
                    foot_step = walk.setGoalPos()

        for id in range(p.getNumJoints(RobotId)):
            if name == "PAI":
                p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[id])
            elif name == "MOS":
                qIndex = p.getJointInfo(RobotId, id)[3]
                if qIndex > -1:
                    if 'leg' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'):
                        # R_leg_1 to L_leg_6: 15-26
                        p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force=5)
        p.stepSimulation()
        step += 1
        if step % 50 == 0:
            print("step is "+str(step))
        # """""
p.disconnect()

plt.figure("joint_angles")
for i in range(10):
    plt.subplot(2, 5, i+1)
    plt.plot(joint_angles_recorder[:, i], label="joint {} angle".format(i))
    plt.legend()

plt.show()
# sleep(TIME_STEP) # delete -> speed up
# print(f'total walking time:{time.time()-t1}')

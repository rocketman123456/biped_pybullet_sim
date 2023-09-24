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
        robot_id = p.loadURDF("urdf/urdf/thmos_mix.urdf", [0, 0, 0.43])  # 0.43
    elif name == "PAI":
        robot_id = p.loadURDF("urdf/PAI-urdf/urdf/PAI-urdf.urdf", [0, 0, 0.36])  # 0.43

    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=60, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    index = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        index[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = id
        joint_info = p.getJointInfo(robot_id, id)
        # joint_name = joint_info[1]#.decode("utf-8")
        # parent_link_index = joint_info[16]
        # joint_axis = joint_info[13]
        print(f"Joint Name: {id} / {joint_info}")

    z = 0.30
    if name == "MOS":
        left_foot0 = p.getLinkState(robot_id, index['L_leg_6_link'])[0]
        right_foot0 = p.getLinkState(robot_id, index['R_leg_6_link'])[0]
        z = 0.30
    elif name == "PAI":
        left_foot0 = p.getLinkState(robot_id, index['left-ankle'])[0]
        right_foot0 = p.getLinkState(robot_id, index['right-ankle'])[0]
        z = 0.40
    print(f'state:\nleft:{left_foot0}\nright:{right_foot0}')

    # """
    joint_angles = []
    for id in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, id)[3] > -1:
            joint_angles += [0,]

    left_foot = left_foot0
    right_foot = right_foot0
    # left_foot  = [ left_foot0[0]-0.0,  left_foot0[1]+0.01,  left_foot0[2]-0.04]
    # right_foot = [right_foot0[0]-0.0, right_foot0[1]-0.01, right_foot0[2]-0.04]
    print("current robot is {}, z is {}".format(name, z))
    pc = preview_control(0.01, 1.0, z)
    walk = walking(left_foot, right_foot, joint_angles, pc, name)

    index_dof = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        print(f"{id}, {p.getJointInfo(robot_id, id)[12].decode('UTF-8')}")
        index_dof[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = p.getJointInfo(robot_id, id)[3] - 7

    # goal position (x, y) theta
    t1 = time.time()
    # print("\n\n ==n is null==\n")
    foot_step = walk.setGoalPos([1.5, 0.0, 0])
    print(f'set goal time:{time.time()-t1}')

    # print("foot_step:", foot_step)

    # init position
    # init_pos = [-0.09,0.0,-0.49,1.11,-0.61,0.09,0.0,0.49,-1.11,0.62]
    # init_pos = [0.0,0.0,-0.29,1.11,-0.61,0.0,0.0,0.29,-1.11,0.62]
    init_pos, _, _, _, _ = walk.getNextPos()
    for i in range(200):
        p.stepSimulation()
    for t in range(200):
        # if t % 20 == 0:
            # print("time is "+str(t))
        current_pos = [angle*t/200 for angle in init_pos]
        for i in range(len(init_pos)):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, current_pos[i], force=2)
        p.stepSimulation()
    joint_angles = init_pos
    # for i in range(10000):
    #   p.stepSimulation()
    j = 0
    step = 0
    joint_angles_recorder = np.array([np.nan for i in range(10)])
    while p.isConnected():
        # contact_points = p.getContactPoints(robot_id, -1)
        # if len(contact_points)<=0:
        #   # print("robot isn't in contact with the ground.")
        #   p.stepSimulation()
        #   continue
        j += 1
        if j >= 10:
            joint_angles, lf, rf, xp, n = walk.getNextPos()
            joint_angles_recorder = np.row_stack((joint_angles_recorder, np.array(joint_angles)))
            # print(f'\n-> {step}_joint_angles is:{joint_angles}/{lf}/{rf}/{n}\n')
            j = 0
            if n == 0:
                if (len(foot_step) <= 5):
                    x_goal, y_goal, th = random()-0.5, random()-0.5, random()-0.5
                    break
                    foot_step = walk.setGoalPos([x_goal, y_goal, th])
                else:
                    # print("\n\n ==n is null==\n")
                    foot_step = walk.setGoalPos()

        for id in range(p.getNumJoints(robot_id)):
            if name == "PAI":
                p.setJointMotorControl2(robot_id, id, p.POSITION_CONTROL, joint_angles[id])
            elif name == "MOS":
                qIndex = p.getJointInfo(robot_id, id)[3]
                if qIndex > -1:
                    if 'leg' in p.getJointInfo(robot_id, id)[1].decode('UTF-8'):
                        # R_leg_1 to L_leg_6: 15-26
                        p.setJointMotorControl2(robot_id, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force=5)
        p.stepSimulation()
        step += 1
        # if step % 50 == 0:
            # print("step is "+str(step))
p.disconnect()

plt.figure("joint_angles")
for i in range(10):
    plt.subplot(2, 5, i+1)
    plt.plot(joint_angles_recorder[:, i], label="joint {} angle".format(i))
    plt.legend()

plt.show()
# sleep(TIME_STEP) # delete -> speed up
# print(f'total walking time:{time.time()-t1}')

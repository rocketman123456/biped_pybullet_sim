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

if __name__ == '__main__':
    print('Start PI-bot walking simulation')

    # setup simulation parameters
    TIME_STEP = 0.001
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)
    # name = "MOS"
    name = "PAI"

    # load urdf model for simulation
    plane_id = p.loadURDF("plane.urdf", [0, 0, 0])
    robot_id = p.loadURDF("urdf/PAI-urdf/urdf/PAI-urdf.urdf", [0, 0, 0.36])  # 0.43

    # setup visualize camera
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=60, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    # print robot link info
    index = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        index[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = id
        joint_info = p.getJointInfo(robot_id, id)
        # joint_name = joint_info[1]#.decode("utf-8")
        # parent_link_index = joint_info[16]
        # joint_axis = joint_info[13]
        print(f"Joint Name: {id} / {joint_info}")

    # print robot foot info
    left_foot = p.getLinkState(robot_id, index['left-ankle'])[0]
    right_foot = p.getLinkState(robot_id, index['right-ankle'])[0]
    print(f'state:\n\tleft:{left_foot}\n\tright:{right_foot}')

    # init joint angles
    joint_angles = []
    for id in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, id)[3] > -1:
            joint_angles += [0,]
    print(f"joint count: {len(joint_angles)}")
    print(f"joint angles: {joint_angles}")

    # set walking height
    z = 0.40
    print("current robot is {}, z is {}".format(name, z))

    # find robot dof
    index_dof = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        index_dof[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = p.getJointInfo(robot_id, id)[3] - 7
    print(index_dof)

    print("--------------------------------------------------------------")
    print("--------------------------------------------------------------")
    print("--------------------------------------------------------------")

    # create robot and controller model

    # wait a period
    for i in range(500):
        p.stepSimulation()
        # time.sleep(0.0001)

    # step simulation loop
    while p.isConnected():
        # biped control logic
        # for i in range(sizeof(index_dof)):
            # print()
        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, math.pi / 4, force = 3.0)
        p.setJointMotorControl2(robot_id, 5, p.POSITION_CONTROL, math.pi / 4, force = 3.0)
        p.stepSimulation()
        # time.sleep(0.0001)

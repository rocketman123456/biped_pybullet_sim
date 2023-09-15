from walking.kinematics import *
from walking.mos_leg_ik import *
import pybullet_data
import pybullet as p

# unable to run, have some bugs
if __name__ == '__main__':
    TIME_STEP = 0.001
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)
    # setup visualize camera
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=60, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    plane_id = p.loadURDF("plane.urdf", [0, 0, 0])
    # robot_id = p.loadURDF("urdf/MOS-urdf/thmos_mix.urdf", [0, 0, 0])
    robot_id = p.loadURDF("urdf/PAI-urdf/urdf/PAI-urdf.urdf", [0, 0, 0.4])
    # robot_id = p.loadURDF("../URDF/urdf/gankenkun.urdf", [0, 0, 0])
    # kine = kinematics(robot_id)
    kine = THMOSLegIK()

    # print robot link info
    index = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        index[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = id
        joint_info = p.getJointInfo(robot_id, id)
        print(f"Joint Name: {id} / {joint_info}")

    # left_foot_pos0,  left_foot_ori0 = p.getLinkState(robot_id, index['left_foot_link'])[:2]
    # right_foot_pos0, right_foot_ori0 = p.getLinkState(robot_id, index['right_foot_link'])[:2]
    left_foot_pos0,  left_foot_ori0 = p.getLinkState(robot_id, index['left-ankle'])[:2]
    right_foot_pos0, right_foot_ori0 = p.getLinkState(robot_id, index['right-ankle'])[:2]

    index_dof = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        index_dof[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = p.getJointInfo(robot_id, id)[3] - 7

    joint_angles = []
    for id in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, id)[3] > -1:
            joint_angles += [0,]

    height = 0.0
    velocity = 0.1

    left_angles = kine.LegIKMove('Left', [left_foot_pos0[0], left_foot_pos0[1], height, 0.0, 0.0, 0.0])
    right_angles = kine.LegIKMove('Right', [right_foot_pos0[0], right_foot_pos0[0], height, 0.0, 0.0, 0.0])
    print(f"left ik angles: {left_angles}")
    print(f"right ik angles: {right_angles}")

    while p.isConnected():
        height += velocity * TIME_STEP

        left_angles = kine.LegIKMove('Left', [0.0, 0.0, 0.1 + height, 0.0, 0.0, 0.0])
        right_angles = kine.LegIKMove('Right', [0.0, 0.0, 0.1 + height, 0.0, 0.0, 0.0])
        joint_angles = left_angles[0:5] + right_angles[0:5]
        print(joint_angles)

        for id in range(p.getNumJoints(robot_id)):
            qIndex = p.getJointInfo(robot_id, id)[3]
            if qIndex > -1:
                p.setJointMotorControl2(robot_id, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force=5)
                # p.setJointMotorControl2(robot_id, id, p.POSITION_CONTROL, joint_angles[qIndex-7], force=6.0)

        if height <= 0.0 or height >= 0.1:
            velocity *= -1.0

        p.stepSimulation()

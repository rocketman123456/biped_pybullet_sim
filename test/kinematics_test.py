from walking.kinematics import *
import pybullet as p

# unable to run, have some bugs
if __name__ == '__main__':
    TIME_STEP = 0.001
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)

    # plane_id = p.loadURDF("plane.urdf", [0, 0, 0])
    robot_id = p.loadURDF("urdf/MOS-urdf/thmos_mix.urdf", [0, 0, 0])
    # robot_id = p.loadURDF("urdf/PAI-urdf/urdf/PAI-urdf.urdf", [0, 0, 0])
    # robot_id = p.loadURDF("../URDF/urdf/gankenkun.urdf", [0, 0, 0])
    kine = kinematics(robot_id)

    index = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        index[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = id

    # left_foot_pos0,  left_foot_ori0 = p.getLinkState(robot_id, index['left_foot_link'])[:2]
    # right_foot_pos0, right_foot_ori0 = p.getLinkState(robot_id, index['right_foot_link'])[:2]
    left_foot_pos0,  left_foot_ori0 = p.getLinkState(robot_id, index['left_foot_link'])[:2]
    right_foot_pos0, right_foot_ori0 = p.getLinkState(robot_id, index['right_foot_link'])[:2]

    index_dof = {p.getBodyInfo(robot_id)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(robot_id)):
        index_dof[p.getJointInfo(robot_id, id)[12].decode('UTF-8')] = p.getJointInfo(robot_id, id)[3] - 7

    joint_angles = []
    for id in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, id)[3] > -1:
            joint_angles += [0,]

    height = 0.0
    velocity = 0.1

    while p.isConnected():
        height += velocity * TIME_STEP
        body_pos, body_ori = p.getLinkState(robot_id, index['body_link'])[:2]
        tar_left_foot_pos = [left_foot_pos0[0], left_foot_pos0[1], height, 0.0, 0.0, 0.0]
        tar_right_foot_pos = [right_foot_pos0[0], right_foot_pos0[1], height, 0.0, 0.0, 0.0]
        joint_angles = kine.solve_ik(tar_left_foot_pos, tar_right_foot_pos, joint_angles)

        for id in range(p.getNumJoints(robot_id)):
            qIndex = p.getJointInfo(robot_id, id)[3]
            if qIndex > -1:
                p.setJointMotorControl2(robot_id, id, p.POSITION_CONTROL, joint_angles[qIndex-7])

        if height <= 0.0 or height >= 0.1:
            velocity *= -1.0

        p.stepSimulation()

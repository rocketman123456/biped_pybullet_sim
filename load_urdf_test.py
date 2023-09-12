import pybullet as p
import pybullet_data

if __name__ == '__main__':
    TIME_STEP = 0.001
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)

    planeId = p.loadURDF("plane.urdf", [0, 0, 0])
    RobotId = p.loadURDF("urdf/MOS-urdf/thmos_mix.urdf", [0, 0, 0.43])
    # RobotId = p.loadURDF("urdf/PAI-urdf/urdf/PAI-urdf.urdf", [0, 0, 0.36])

    for id in range(p.getNumJoints(RobotId)):
        print(p.getJointInfo(RobotId, id))

    while p.isConnected():
        p.stepSimulation()

import pybullet as p
import time
import pybullet_data
import numpy as np
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.2]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("Assem1/urdf/base.urdf",cubeStartPos,useFixedBase=1)
boxId2 = p.loadURDF("Assem1/urdf/base.urdf",cubeStartPos)
# boxId2 = p.loadURDF("Assem1/urdf/Assem1.urdf",[0.5,0,1], cubeStartOrientation,useFixedBase=1)


# p.createConstraint(boxId2, -1, boxId2, 1, p.JOINT_PRISMATIC, [0, 0, 0], [0, 0, 0], [0, 0, 0])

p.createConstraint(boxId, -1, boxId2, -1, p.JOINT_PRISMATIC, [0, 1, 0], [0, 0, 0], [0, 0, 0])




for i in range (10000):
    action = 1.5*np.sin(i/200*np.pi*2)
    # p.setJointMotorControl2(boxId,0,controlMode = p.POSITION_CONTROL,targetPosition = action)
    # p.setJointMotorControl2(boxId2,0,controlMode = p.POSITION_CONTROL,targetPosition = action)
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

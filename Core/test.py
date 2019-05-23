from kinematics import Kinematic
import numpy as np
import pybullet as p
import pybullet_data
import time
import math

useMaximalCoordinates = False
useRealTime = 1
fixedTimeStep = 1. / 100
numSolverIterations = 50
t = 0.0
t_end = t + 100
physId = p.connect(p.SHARED_MEMORY)
if (physId < 0):
  p.connect(p.GUI)
angle = 0   
orn = p.getQuaternionFromEuler([0, angle, 0])
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeUid=p.loadURDF("plane.urdf", [0, 0, 0], orn)
texUid = p.loadTexture("concrete.png")
#texUid = p.loadTexture("asphalt.png")
#p.changeVisualShape(planeUid, -1, rgbaColor=[1, 1, 1, 0.5])
p.changeVisualShape(planeUid, -1, textureUniqueId=texUid)
p.setTimeOut(4000000)

p.setGravity(0, 0, -9.81)
p.setTimeStep(fixedTimeStep)

orn = p.getQuaternionFromEuler([0, 0, 0.0])
p.setRealTimeSimulation(useRealTime)
quadruped = p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", [1, -1, .3],
                       orn,
                       useFixedBase=False,
                       useMaximalCoordinates=useMaximalCoordinates,
                       flags=p.URDF_USE_IMPLICIT_CYLINDER)
nJoints = p.getNumJoints(quadruped)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(quadruped, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]


front_left_shoulder = jointNameToId['front_left_shoulder']
front_left_leg = jointNameToId['front_left_leg']
front_left_foot = jointNameToId['front_left_foot']
front_left_toe = jointNameToId['front_left_toe']

front_right_shoulder = jointNameToId['front_right_shoulder']
front_right_leg = jointNameToId['front_right_leg']
front_right_foot = jointNameToId['front_right_foot']
front_right_toe = jointNameToId['front_right_toe']

rear_left_shoulder = jointNameToId['rear_left_shoulder']
rear_left_leg = jointNameToId['rear_left_leg']
rear_left_foot = jointNameToId['rear_left_foot']
rear_left_toe = jointNameToId['rear_left_toe']

rear_right_shoulder = jointNameToId['rear_right_shoulder']
rear_right_leg = jointNameToId['rear_right_leg']
rear_right_foot = jointNameToId['rear_right_foot']
rear_right_toe = jointNameToId['rear_right_toe']
Lp=np.array([[100,-100,100,1],[100,-100,-100,1],[-100,-100,100,1],[-100,-100,-100,1]])
kin=Kinematic()
p.setRealTimeSimulation(useRealTime)
kp = 0.05
kd = .1
maxForce = 3.5
ref_time = time.time()
for i in range (nJoints):
	p.changeDynamics(quadruped,i,localInertiaDiagonal=[0.000001,0.000001,0.000001])

while (1):

    if (useRealTime):
        t = time.time() - ref_time
    else:
        t = t + fixedTimeStep
    cubePos, cubeOrn = p.getBasePositionAndOrientation(quadruped)
    #print(cubePos,cubeOrn)

    for lx,leg in enumerate(['front_left','front_right','rear_left','rear_right']):
        for px,part in enumerate(['shoulder','leg','foot']):
            j=jointNameToId[leg+"_"+part]
            p.setJointMotorControl2(bodyIndex=quadruped,
                            jointIndex=j,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition= 0,
                            positionGain=kp,
                            velocityGain=kd,
                            force=maxForce)
    #Lp[0][0]+=0.2
    pia=1*math.sin((t/10))
    angles=kin.calcIK(Lp,(0,0,0),(0,0,0))
    #joints=[jointNameToId[x+"_"+a] for x in ['front_left','front_right','rear_left','rear_right']  for a in ['shoulder','leg','foot']  ]
    idx=0
    for lx,leg in enumerate(['front_left','front_right','rear_left','rear_right']):
        for px,part in enumerate(['shoulder','leg','foot']):
            j=jointNameToId[leg+"_"+part]
            aa=(angles[lx][px])
#            if(px==0 and lx==0):
#                print(aa)
            """
            p.setJointMotorControl2(bodyIndex=quadruped,
                                    jointIndex=j,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition= math.pi,
                                    positionGain=kp,
                                    velocityGain=kd,
                                    force=maxForce)
            """
            idx+=1

    if (useRealTime == 0):
        p.stepSimulation()
        time.sleep(fixedTimeStep)
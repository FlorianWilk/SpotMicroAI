import pybullet as p
import pybullet_data
import time
import math


def drawInertiaBox(parentUid, parentLinkIndex, color):
  dyn = p.getDynamicsInfo(parentUid, parentLinkIndex)
  mass = dyn[0]
  frictionCoeff = dyn[1]
  inertia = dyn[2]
  if (mass > 0):
    Ixx = inertia[0]
    Iyy = inertia[1]
    Izz = inertia[2]
    boxScaleX = 0.5 * math.sqrt(6 * (Izz + Iyy - Ixx) / mass)
    boxScaleY = 0.5 * math.sqrt(6 * (Izz + Ixx - Iyy) / mass)
    boxScaleZ = 0.5 * math.sqrt(6 * (Ixx + Iyy - Izz) / mass)

    halfExtents = [boxScaleX, boxScaleY, boxScaleZ]
    pts = [[halfExtents[0], halfExtents[1], halfExtents[2]],
           [-halfExtents[0], halfExtents[1], halfExtents[2]],
           [halfExtents[0], -halfExtents[1], halfExtents[2]],
           [-halfExtents[0], -halfExtents[1], halfExtents[2]],
           [halfExtents[0], halfExtents[1], -halfExtents[2]],
           [-halfExtents[0], halfExtents[1], -halfExtents[2]],
           [halfExtents[0], -halfExtents[1], -halfExtents[2]],
           [-halfExtents[0], -halfExtents[1], -halfExtents[2]]]
    """
    p.addUserDebugLine(pts[0],
                       pts[1],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[1],
                       pts[3],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[3],
                       pts[2],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[2],
                       pts[0],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)

    p.addUserDebugLine(pts[0],
                       pts[4],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[1],
                       pts[5],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[2],
                       pts[6],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[3],
                       pts[7],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)

    p.addUserDebugLine(pts[4 + 0],
                       pts[4 + 1],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[4 + 1],
                       pts[4 + 3],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[4 + 3],
                       pts[4 + 2],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[4 + 2],
                       pts[4 + 0],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    """

toeConstraint = True
useMaximalCoordinates = False
useRealTime = 0

#the fixedTimeStep and numSolverIterations are the most important parameters to trade-off quality versus performance
fixedTimeStep = 1. / 100
numSolverIterations = 50

if (useMaximalCoordinates):
  fixedTimeStep = 1. / 500
  numSolverIterations = 200

speed = 10
amplitude = 0.8
jump_amp = 0.5
maxForce = 3.5
kneeFrictionForce = 0
kp = 1
kd = .5
maxKneeForce = 1000

physId = p.connect(p.SHARED_MEMORY)
if (physId < 0):
  p.connect(p.GUI)
#p.resetSimulation()

angle = 0  # pick in range 0..0.2 radians
orn = p.getQuaternionFromEuler([0, angle, 0])
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0], orn)
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                    "genericlogdata.bin",
                    maxLogDof=16,
                    logFlags=p.STATE_LOG_JOINT_TORQUES)
p.setTimeOut(4000000)

#p.setGravity(0, 0, 0)
p.setGravity(0, 0, -9.81)
p.setTimeStep(fixedTimeStep)

orn = p.getQuaternionFromEuler([0, 0, 0.4])
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


#fixtorso = p.createConstraint(-1,-1,quadruped,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])

motordir = [-1, -1, -1, -1, 1, 1, 1, 1]
halfpi = 1.57079632679
twopi = 4 * halfpi
kneeangle = -2.1834

dyn = p.getDynamicsInfo(quadruped, -1)
mass = dyn[0]
friction = dyn[1]
localInertiaDiagonal = dyn[2]

print("localInertiaDiagonal", localInertiaDiagonal)

#this is a no-op, just to show the API
p.changeDynamics(quadruped, -1, localInertiaDiagonal=localInertiaDiagonal)
for foot in (front_left_foot,front_right_foot,rear_left_foot,rear_right_foot):
    p.setJointMotorControl2(bodyIndex=quadruped,
                            jointIndex=foot,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition= 2.47,
                            positionGain=kp,
                            velocityGain=kd,
                            force=maxForce)
for leg in (front_left_leg,front_right_leg,rear_left_leg,rear_right_leg):
    p.setJointMotorControl2(bodyIndex=quadruped,
                            jointIndex=leg,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition= -1.5,
                            positionGain=kp,
                            velocityGain=kd,
                            force=maxForce)

#for i in range (nJoints):
#	p.changeDynamics(quadruped,i,localInertiaDiagonal=[0.000001,0.000001,0.000001])



t = 0.0
t_end = t + 15
ref_time = time.time()
while (t < t_end):
  p.setGravity(0, 0, -10)
  if (useRealTime):
    t = time.time() - ref_time
  else:
    t = t + fixedTimeStep
  if (useRealTime == 0):
    p.stepSimulation()
    time.sleep(fixedTimeStep)

print("quadruped Id = ")
print(quadruped)
p.saveWorld("quadru.py")
logId = p.startStateLogging(p.STATE_LOGGING_MINITAUR, "quadrupedLog.bin", [quadruped])
p.setRealTimeSimulation(useRealTime)
#jump
t = 0.0
t_end = t + 100
i = 0
ref_time = time.time()

while (1):
  if (useRealTime):
    t = time.time() - ref_time
  else:
    t = t + fixedTimeStep
  if (True):
    target = math.sin(t * speed) * jump_amp + 1.57
   
  if (useRealTime == 0):
    p.stepSimulation()
    time.sleep(fixedTimeStep)


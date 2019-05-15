import numpy as np
import random
import tensorflow as tf
import matplotlib.pyplot as plt
import scipy.misc
import os
import csv
import itertools
import tensorflow.contrib.slim as slim
from tensorflow.keras import layers
import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from collections import deque

from dqn_agent import Agent

# See https://arxiv.org/pdf/1804.10332.pdf
# Observation-Space -> roll,pitch,vax,vay,vaz,motor1-12
# The Observation-Space includes the additional Servos for Shoulders
#
# Action-Space -> Leg1-4 position in Leg Space (swing, extension, shift)
# The Action-Space includes one additional Information "Shift", because SpotMicro also has a Shoulder

state_size=17
action_size=12

# original DQN-Code by Jonas Leininger https://github.com/JonasLeininger/deep-q-network-banana-navigation

agent = Agent(state_size=state_size, action_size=action_size)

useMaximalCoordinates = False
useRealTime = 1
fixedTimeStep = 1. / 100
numSolverIterations = 50


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
#p.changeDynamics(quadruped, -1, localInertiaDiagonal=localInertiaDiagonal)
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

for i in range (nJoints):
	p.changeDynamics(quadruped,i,localInertiaDiagonal=[0.000001,0.000001,0.000001])

t = 0.0
t_end = t + 15
ref_time = time.time()

print("quadruped Id = ")
print(quadruped)
#p.saveWorld("quadru.py")
#logId = p.startStateLogging(p.STATE_LOGGING_MINITAUR, "quadrupedLog.bin", [quadruped])
p.setRealTimeSimulation(useRealTime)
#jump
t = 0.0
t_end = t + 100
i = 0
ref_time = time.time()

while (1):
    motors=(front_left_shoulder,front_right_shoulder,rear_left_shoulder,rear_right_shoulder,front_left_leg,front_right_leg,rear_left_leg,rear_right_leg,front_left_foot,front_right_foot,rear_left_foot,rear_right_foot)
    motor_torques = [
          p.getJointState(quadruped,jointIndex=motor_id)[3] for motor_id in motors
      ]

    # TODO: IMU Inputs
    state=[1,2,3,4,5]

    state.extend(motor_torques)
    state = np.reshape(state, [1, state_size])
    action = agent.act(state)

    if (useRealTime):
        t = time.time() - ref_time
    else:
        t = t + fixedTimeStep

    idx=0
    for leg in motors:
        # TODO: Convert to Legspace
        p.setJointMotorControl2(bodyIndex=quadruped,
                            jointIndex=leg,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition= action[0][idx],
                            positionGain=kp,
                            velocityGain=kd,
                            force=maxForce)
    idx+=1

    if (useRealTime == 0):
        p.stepSimulation()
        time.sleep(fixedTimeStep)


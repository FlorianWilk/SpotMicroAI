"""
Simulation of SpotMicroAI and it's Kinematics 
Use a Gamepad to see how it works
"""

from kinematics import Kinematic
import numpy as np
import pybullet as p
import pybullet_data
import time
import math
from inputs import devices, get_gamepad
from thinputs import ThreadedInputs
import matplotlib.pyplot as plt

# Simulation Configuration
useMaximalCoordinates = False
useRealTime = True
fixedTimeStep = 1. / 100
numSolverIterations = 200

# Parameters for Servos - still wrong
kp = 0.012
kd = .2
maxForce = 12.5

physId = p.connect(p.SHARED_MEMORY)
if (physId < 0):
    p.connect(p.GUI)
angle = 90

def loadModels():
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
    p.setTimeStep(fixedTimeStep)

    orn = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeUid = p.loadURDF("plane.urdf", [0, 0, 0], orn)
    texUid = p.loadTexture("concrete.png")
    p.changeVisualShape(planeUid, -1, textureUniqueId=texUid)

    orn = p.getQuaternionFromEuler([0, 0, 90.0])
    p.setRealTimeSimulation(useRealTime)
    quadruped = p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", [0, 0, 0.5],
                           orn,
                           useFixedBase=False,
                           useMaximalCoordinates=useMaximalCoordinates,
                           flags=p.URDF_USE_IMPLICIT_CYLINDER)
    return quadruped

def changeDynamics(quadruped):
    nJoints = p.getNumJoints(quadruped)
    for i in range(nJoints):
        p.changeDynamics(quadruped, i, localInertiaDiagonal=[
                         0.000001, 0.000001, 0.000001])

def getJointNames(quadruped):
    nJoints = p.getNumJoints(quadruped)
    jointNameToId = {}

    for i in range(nJoints):
        jointInfo = p.getJointInfo(quadruped, i)
        jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    return jointNameToId

def addInfoText(text):
    return p.addUserDebugText(text, [0, 0, 0.45], textColorRGB=[1, 1, 1], textSize=1.5, parentObjectUniqueId=quadruped, parentLinkIndex=1)

def handleCamera(cubePos, cubeOrn):
    init_camera_vector = (-1, 0, 0)
    init_up_vector = (0, 0, 1)
    # Rotate vectors with body
    rot_matrix = p.getMatrixFromQuaternion(cubeOrn)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    addV = (30, 0, 0)
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(
        cubePos + 0.15 * camera_vector, cubePos + 3 * camera_vector,  up_vector)
    img = p.getCameraImage(320, 200, view_matrix, projection_matrix)

def checkSimulationReset():
    global quadruped
    rot = p.getEulerFromQuaternion(bodyOrn)
    (xr, yr, zr) = rot
    if(abs(xr) > math.pi/2 or abs(yr) > math.pi/2):
        p.resetSimulation()
        quadruped = loadModels()
        return True
    return False

def initPlot():
    figure = plt.figure(figsize=[15, 4.5])
    figure.subplots_adjust(left=0.05, bottom=0.11,
                           right=0.97, top=0.9, wspace=0.4, hspace=0.55)

    ax_pos = figure.add_subplot(141)
    ax_pos.set_title("Joint Position")
    ax_pos.plot(t, q_tor[0], '--r', lw=4, label='Desired q0')
    ax_pos.plot(t, q_tor[1], '--b', lw=4, label='Desired q1')
    ax_pos.set_ylim(-1., 1.)
    ax_pos.legend()
    plt.pause(0.01)

testGamePad = False
if testGamePad:
    while 1:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)

# Matplotlib - coming soon
plot = False

start = 0.0
end = 1.0
delta_t = 0.0001
steps = int((end - start) / delta_t)
q_tor = [[0.] * steps, [0.] * steps]
t = [0] * steps
t = 0.0
t_end = t + 100

# Gamepad Initialisation
# Dictionary of game controller buttons we want to include.
gamepadInputs = {'ABS_X': 128, 'ABS_RZ': 127,
                 'ABS_Y': 126, 'ABS_Z': 125,

                 # right side of gamepad
                 'BTN_TIGGER': 124, 'BTN_THUMB': 123, 'BTN_THUMB2': 122, 'BTN_TOP': 121,

                 # left
                 'ABS_HAT0X': 120, 'ABS_HAT0Y': 119,
                 # left top
                 'BTN_TOP2': 118, 'BTN_BASE': 117,

                 # right top
                 'BTN_PINKIE': 116, 'BTN_BASE2': 115
                 }

joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

# Initialise the gamepad object using the gamepad inputs Python package
gamepad = ThreadedInputs()
for gamepadInput in gamepadInputs:
    gamepad.append_command(gamepadInput, gamepadInputs[gamepadInput])
gamepad.start()

def handleGamepad():
    global joy_x, joy_z, joy_y, joy_rz
    commandInput, commandValue = gamepad.read()
    # Gamepad button command filter
    if commandInput == 'ABS_X':
        joy_x = commandValue
    if commandInput == 'ABS_Y':
        joy_y = commandValue
    if commandInput == 'ABS_Z':
        joy_z = commandValue
    if commandInput == 'ABS_RZ':
        joy_rz = commandValue


if plot:
    initPlot()

# Main

IDheight = p.addUserDebugParameter("height", -40, 60, 0)
IDroll = p.addUserDebugParameter("roll", -20, 20, 0)

quadruped = loadModels()
changeDynamics(quadruped)
jointNameToId = getJointNames(quadruped)

# Initial FootPositions lf,rf,lb,rb and directions of motors (lf and lb had to be inverted)
Lp = np.array([[120, -100, 140, 1], [120, -100, -140, 1],
               [-120, -100, 140, 1], [-120, -100, -140, 1]])
dirs = [[-1, 1, 1], [1, 1, 1], [-1, 1, 1], [1, 1, 1]]

kin = Kinematic()

p.setRealTimeSimulation(useRealTime)
ref_time = time.time()

textId = addInfoText("")

# Camera Settings
fov, aspect, nearplane, farplane = 90, 1.3, .0111, 100
projection_matrix = p.computeProjectionMatrixFOV(
    fov, aspect, nearplane, farplane)

while True:

    bodyPos, bodyOrn = p.getBasePositionAndOrientation(quadruped)
    linearVel, angularVel = p.getBaseVelocity(quadruped)

    oldTextId = textId
    textId = addInfoText("{:.1f} m Distance".format(
        math.sqrt(bodyPos[0]**2+bodyPos[1]**2)))
    p.removeUserDebugItem(oldTextId)

    height = p.readUserDebugParameter(IDheight)
    roll = p.readUserDebugParameter(IDroll)

    handleCamera(bodyPos, bodyOrn)
    handleGamepad()

    angles = kin.calcIK(Lp, (math.pi/180*roll, 0.6/256*joy_x-0.3, -(0.9/256*joy_y-0.45)),  # (0,20,0))
                        (100/256*-joy_rz-20+100, 40+height, 60/256*joy_z-30))

    if checkSimulationReset():
        continue

    for lx, leg in enumerate(['front_left', 'front_right', 'rear_left', 'rear_right']):
        for px, part in enumerate(['shoulder', 'leg', 'foot']):
            j = jointNameToId[leg+"_"+part]
            p.setJointMotorControl2(bodyIndex=quadruped,
                                    jointIndex=j,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=angles[lx][px]*dirs[lx][px],
                                    positionGain=kp,
                                    velocityGain=kd,
                                    force=maxForce)

    if (useRealTime):
        t = time.time() - ref_time
    else:
        t = t + fixedTimeStep

    if (useRealTime == False):
        p.stepSimulation()
        time.sleep(fixedTimeStep)

gamepad.stop()

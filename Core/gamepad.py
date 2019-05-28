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

INIT_ORIENTATION=p.getQuaternionFromEuler([0, 0, 90.0])
INIT_POSITION=[0, 0, 0.5]

REFLECTION=False

def loadModels():
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
    p.setTimeStep(fixedTimeStep)

    orn = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeUid = p.loadURDF("plane_transparent.urdf", [0, 0, 0], orn)
    texUid = p.loadTexture("concrete.png")
    p.changeVisualShape(planeUid, -1, textureUniqueId=texUid)

    stairsUid = p.loadURDF("../urdf/stairs_gen.urdf.xml", [0, -1, 0], orn)

    p.setRealTimeSimulation(useRealTime)
    quadruped = p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", INIT_POSITION,
                           INIT_ORIENTATION,
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

oldTextId=0
textId=0
oldDebugInfo=[]

def addInfoText(bodyPos,bodyEuler,linearVel,angularVel):
    global textId,oldDebugInfo
    text="Distance: {:.1f}m".format(math.sqrt(bodyPos[0]**2+bodyPos[1]**2))
    text2="Roll/Pitch: {:.1f} / {:.1f}".format(math.degrees(bodyEuler[0]),math.degrees(bodyEuler[1]))
    text3="Vl: {:.1f} / {:.1f} / {:.1f} Va: {:.1f} / {:.1f} / {:.1f}".format(linearVel[0],linearVel[1],linearVel[2],
        angularVel[0],angularVel[1],angularVel[2])
    x,y=bodyPos[0],bodyPos[1]
    newDebugInfo=[
    p.addUserDebugLine([x, y, 0], [x, y, 1], [0,1,0]),
    p.addUserDebugText(text, [x+0.03, y, 0.6], textColorRGB=[1, 1, 1], textSize=1.0),
    p.addUserDebugText(text2, [x+0.03, y, 0.5], textColorRGB=[1, 1, 1], textSize=1.0),
    p.addUserDebugText(text3, [x+0.03, y, 0.4], textColorRGB=[1, 1, 1], textSize=1.0)]
    p.addUserDebugLine([-0.3, 0, 0], [0.3, 0, 0], [0,1,0], parentObjectUniqueId=quadruped, parentLinkIndex=1 ),
    p.addUserDebugLine([0, -0.2, 0], [0, 0.2, 0], [0,1,0], parentObjectUniqueId=quadruped, parentLinkIndex=1 ),

    if len(oldDebugInfo)>0:
        for x in oldDebugInfo:
            p.removeUserDebugItem(x)
    oldDebugInfo=newDebugInfo


def handleCamera(cubePos, cubeOrn):
    init_camera_vector = (-1, 0, 0)
    init_up_vector = (0, 0, 1)
    # Rotate vectors with body
    orn = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, -math.pi/180*25,0]))
    orn = np.array(orn).reshape(3, 3)
    rot_matrix = p.getMatrixFromQuaternion(cubeOrn)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    camera_vector = rot_matrix.dot(orn.dot(init_camera_vector))
    up_vector = rot_matrix.dot(orn.dot(init_up_vector))
    view_matrix = p.computeViewMatrix(
        cubePos + 0.17 * camera_vector, cubePos + 3 * camera_vector,  up_vector)
    img = p.getCameraImage(320, 200, view_matrix, projection_matrix)

def checkSimulationReset():
    global quadruped
    rot = p.getEulerFromQuaternion(bodyOrn)
    (xr, yr, _) = rot
    if(abs(xr) > math.pi/3 or abs(yr) > math.pi/3):
        #p.resetSimulation()
        p.resetBasePositionAndOrientation(quadruped, INIT_POSITION,INIT_ORIENTATION)
        p.resetBaseVelocity(quadruped, [0, 0, 0], [0, 0, 0])
        #quadruped = loadModels()
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

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

resetPose()

# Initialise the gamepad object using the gamepad inputs Python package
gamepad = ThreadedInputs()
for gamepadInput in gamepadInputs:
    gamepad.append_command(gamepadInput, gamepadInputs[gamepadInput])
gamepad.start()

def handleGamepad():
    # TODO: globals are bad
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
    if commandInput == 'BTN_TOP2':
        resetPose()

if plot:
    initPlot()

# Main
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
if REFLECTION:
    p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 1)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
IDheight = p.addUserDebugParameter("height", -40, 90, 0)
IDroll = p.addUserDebugParameter("roll", -20, 20, 0)
IDkp = p.addUserDebugParameter("Kp", 0, 0.05, 0.012) # 0.05
IDkd = p.addUserDebugParameter("Kd", 0, 1, 0.2) # 0.5
IDmaxForce = p.addUserDebugParameter("MaxForce", 0, 50, 12.5)

quadruped = loadModels()
changeDynamics(quadruped)
jointNameToId = getJointNames(quadruped)
L=140
W=75+5+40
# Initial FootPositions lf,rf,lb,rb and directions of motors (lf and lb had to be inverted)
#Lp = np.array([[120, -100, 140, 1], [120, -100, -140, 1],
#               [-120, -100, 140, 1], [-120, -100, -140, 1]])
Lp = np.array([[120, -100, W/2, 1], [120, -100, -W/2, 1],
               [-50, -100, W/2, 1], [-50, -100, -W/2, 1]])
dirs = [[-1, 1, 1], [1, 1, 1], [-1, 1, 1], [1, 1, 1]]

kin = Kinematic()

p.setRealTimeSimulation(useRealTime)
ref_time = time.time()

# Camera Settings
fov, aspect, nearplane, farplane = 90, 1.3, .0111, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while True:
    if (useRealTime):
        t = time.time() - ref_time
    else:
        t = t + fixedTimeStep

    if (useRealTime == False):
        p.stepSimulation()
        time.sleep(fixedTimeStep)

    """
    a=math.cos(t*10)*20-100
    b=math.sin(t*10+math.pi)*20-100
    Lp[0][1]=a
    Lp[2][1]=a
    Lp[1][1]=b
    Lp[3][1]=b
    """
    bodyPos, bodyOrn = p.getBasePositionAndOrientation(quadruped)
    linearVel, angularVel = p.getBaseVelocity(quadruped)
    bodyEuler=p.getEulerFromQuaternion(bodyOrn)

    height = p.readUserDebugParameter(IDheight)
    roll = p.readUserDebugParameter(IDroll)
    kp=p.readUserDebugParameter(IDkp)
    kd=p.readUserDebugParameter(IDkd)
    maxForce=p.readUserDebugParameter(IDmaxForce)

    handleCamera(bodyPos, bodyOrn)
    handleGamepad()

    # map the Gamepad Inputs to Pose-Values. Still very hardcoded ranges. 
    # TODO: Make ranges depend on height or smth to keep them valid all the time
    angles = kin.calcIK(Lp, (math.pi/180*roll, 1/256*joy_x-0.5, -(0.9/256*joy_y-0.45)), 
                        (100/256*-joy_rz-20+120, 40+height, 60/256*joy_z-30))

    addInfoText(bodyPos,bodyEuler,linearVel,angularVel)

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


gamepad.stop()

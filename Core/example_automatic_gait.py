"""
Simulation of SpotMicroAI and it's Kinematics 
Use a Gamepad to see how it works
Use Gamepad-Button to switch betweek walk on static-mode
"""
import pybullet as p
import numpy as np
import pybullet_data
import time
import math
import datetime as dt
import matplotlib.animation as animation
import random
from inputs import devices, get_gamepad
from thinputs import ThreadedInputs
import spotmicroai
from kinematicMotion import KinematicMotion,TrottingGait
from environment import environment

rtime=time.time()
env=environment()
def reset():
    global rtime
    rtime=time.time()    

robot=spotmicroai.Robot(False,False,reset)

# TODO: Needs refactoring
speed1=240
speed2=170
speed3=300

speed1=322
speed2=237
speed3=436

spurWidth=robot.W/2+20
stepLength=0
stepHeight=72
iXf=120
iXb=-132
IDspurWidth = p.addUserDebugParameter("spur width", 0, robot.W, spurWidth)
IDstepHeight = p.addUserDebugParameter("step height", 0, 150, stepHeight)

walk=False

# Gamepad Initialisation
# Dictionary of game controller buttons we want to include.
gamepadInputs = {'ABS_X': 128, 'ABS_RZ': 127, 'ABS_Y': 126, 'ABS_Z': 125,
                 'BTN_TIGGER': 124, 'BTN_THUMB': 123, 'BTN_THUMB2': 122, 'BTN_TOP': 121, # right side of gamepad
                 'ABS_HAT0X': 120, 'ABS_HAT0Y': 119, # left
                 'BTN_TOP2': 118, 'BTN_BASE': 117, # left top
                 'BTN_PINKIE': 116, 'BTN_BASE2': 115 # right top
                 }

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

def handleGamepad():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,walk
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



IDheight = p.addUserDebugParameter("height", -40, 90, 40)

Lp = np.array([[iXf, -100,spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

motion=KinematicMotion(Lp)
resetPose()

# Initialise the gamepad object using the gamepad inputs Python package
gamepad = ThreadedInputs()
for gamepadInput in gamepadInputs:
    gamepad.append_command(gamepadInput, gamepadInputs[gamepadInput])
gamepad.start()
trotting=TrottingGait()


s=False
while True:

    bodyPos=robot.getPos()
    bodyOrn,_,_=robot.getIMU()
    xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
    distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)
    if distance>50:
        robot.resetBody()
   
    ir=xr/(math.pi/180)
    handleGamepad()
    d=time.time()-rtime
    height = p.readUserDebugParameter(IDheight)

    # wait 3 seconds to start
    if d>3:
        robot.feetPosition(trotting.positions(d-3))
    else:
        robot.feetPosition(Lp)
    #roll=-xr
    roll=0
    robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
    bodyX=50+yr*10
    robot.bodyPosition((bodyX, 40+height, -ir))
    robot.step()
gamepad.stop()

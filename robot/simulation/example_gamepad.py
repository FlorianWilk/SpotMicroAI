"""
Simulation of SpotMicroAI and it's Kinematics 
Use a Gamepad to see how it works
"""
import pybullet as p
import numpy as np
import pybullet_data
import time
import math
from inputs import devices, get_gamepad
from thinputs import ThreadedInputs
import spotmicroai
import matplotlib.pyplot as plt

# activate to verify the GamePad Controller Values
if False:
    while 1:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)

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

robot=spotmicroai.Robot()
IDheight = p.addUserDebugParameter("height", -40, 90, 0)
IDroll = p.addUserDebugParameter("roll", -20, 20, 0)
#robot.feetPosition(Lp)
resetPose()

# Initialise the gamepad object using the gamepad inputs Python package
gamepad = ThreadedInputs()
for gamepadInput in gamepadInputs:
    gamepad.append_command(gamepadInput, gamepadInputs[gamepadInput])
gamepad.start()

while True:
    handleGamepad()

    height = p.readUserDebugParameter(IDheight)
    roll = p.readUserDebugParameter(IDroll)

    # map the Gamepad Inputs to Pose-Values. Still very hardcoded ranges. 
    # TODO: Make ranges depend on height or smth to keep them valid all the time
    robot.bodyRotation((math.pi/180*roll,1/256*joy_x-0.5,-(0.9/256*joy_y-0.45)))
    robot.bodyPosition((100/256*-joy_rz-20+120, 40+height, 60/256*joy_z-30))
    robot.step()

gamepad.stop()

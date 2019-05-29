"""
Simulation of SpotMicroAI and it's Kinematics 
Use a Gamepad to see how it works
In this example the Bot has a fixed base
"""
import pybullet as p
import numpy as np
import pybullet_data
import time
import math
from inputs import devices, get_gamepad
from thinputs import ThreadedInputs
import spotmicroai
from kinematicMotion import KinematicMotion
import matplotlib.pyplot as plt


robot=spotmicroai.Robot(False,False)
speed1=240
speed2=170
speed3=300
spurWidth=robot.W/2+20
stepLength=87
stepHeight=72
iXf=140
IDspurWidth = p.addUserDebugParameter("spur width", 0, robot.W, spurWidth)
IDstepLength = p.addUserDebugParameter("step length", 0, 200, stepLength)
IDstepHeight = p.addUserDebugParameter("step height", 0, 100, stepHeight)
IDspeed1 = p.addUserDebugParameter("speed 1", 100, 1000, speed1)
IDspeed2 = p.addUserDebugParameter("speed 2", 100, 1000, speed2)
IDspeed3 = p.addUserDebugParameter("speed 3", 100, 1000, speed3)
IDixf = p.addUserDebugParameter("iXf", 0, 400, iXf)


Lp2 = np.array([[120, -100, robot.W, 1], [120, -100, -robot.W, 1],
   [-50, -100, robot.W, 1], [-50, -100, -robot.W, 1]])

Lpm1 = np.array([[100, -50, robot.W/2, 1], [120, -100, -robot.W/2, 1],
   [-70, -50, robot.W/2, 1], [-50, -100, -robot.W/2, 1]])

Lpm2 = np.array([[80, -100, robot.W/2, 1], [120, -100, -robot.W/2, 1],
   [-90, -100, robot.W/2, 1], [-50, -100, -robot.W/2, 1]])

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

def func(p,LLp):
    LLp[1]+=stepHeight*math.sin(math.pi*math.sin(math.pi/2*p))
    return LLp
    #nlp=LLp[2]; LLp[2]+10*math.sin(math.pi*2*p)
    #return [LLp[0],LLp[1],nlp,0]

def func3(p,LLp):
    LLp[0]+=0*math.sin(math.pi*p)
    return LLp
    #nlp=LLp[2]; LLp[2]+10*math.sin(math.pi*2*p)
    #return [LLp[0],LLp[1],nlp,0]
def func2(p,LLp):
    LLp[0]-=0*math.sin(math.pi*p)
    return LLp
    #nlp=LLp[2]; LLp[2]+10*math.sin(math.pi*2*p)
    #return [LLp[0],LLp[1],nlp,0]



def action():
    #motion.moveLegsTo(Lp2,100)
    
    motion.moveLegTo(0,Lpa[0],speed2,func=func)
    motion.moveLegTo(3,Lpa[3],speed2,func=func)
#    time.sleep(500)
    motion.moveLegTo(1,Lpf[1],speed1,func=func3)
    motion.moveLegTo(2,Lpf[2],speed1,func=func2)

def action2():
    motion.moveLegsTo(Lp,400)

def action3():
    #motion.moveLegsTo(Lp2,100)
    
    motion.moveLegTo(0,Lpf[0],speed1,func=func3)
    motion.moveLegTo(3,Lpf[3],speed1,func=func2)
#    time.sleep(500)
    motion.moveLegTo(1,Lpa[1],speed2,func=func)
    motion.moveLegTo(2,Lpa[2],speed2,func=func)

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
    if commandInput == 'BTN_PINKIE':
        if commandValue:
            walk=not walk
    if commandInput == 'BTN_BASE':
        action2()
    if commandInput == 'BTN_BASE2':
        action3()



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

rtime=time.time()
s=False
while True:

    spurWidth = p.readUserDebugParameter(IDspurWidth)
    stepLength = p.readUserDebugParameter(IDstepLength)
    stepHeight = p.readUserDebugParameter(IDstepHeight)
    speed1=p.readUserDebugParameter(IDspeed1)
    speed2=p.readUserDebugParameter(IDspeed2)
    speed3=p.readUserDebugParameter(IDspeed3)
    iXf=p.readUserDebugParameter(IDixf)

    bodyPos=robot.getPos()
    bodyOrn,_,_=robot.getIMU()
    xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
    distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)
    if distance>5:
        robot.resetBody()

    Lpa = np.array([[iXf+stepLength, -100,spurWidth, 1], [iXf+stepLength, -100, -spurWidth, 1],
    [-50+stepLength, -100, spurWidth, 1], [-50+stepLength, -100, -spurWidth, 1]])

    Lpf = np.array([[iXf-stepLength, -100, spurWidth, 1], [iXf-stepLength, -100, -spurWidth, 1],
    [-50-stepLength, -100, spurWidth, 1], [-50-stepLength, -100, -spurWidth, 1]])


    handleGamepad()
    d=time.time()-rtime
    if d>speed3/1000 and walk:
        if(s):
            action3()
            s=False
        else:
            action()
            s=True
        rtime=time.time()


    height = p.readUserDebugParameter(IDheight)

    # map the Gamepad Inputs to Pose-Values. Still very hardcoded ranges. 
    # TODO: Make ranges depend on height or smth to keep them valid all the time
    robot.feetPosition(motion.step())
    roll=-xr*3

    robot.bodyRotation((roll,1/256*joy_x-0.5,-(0.9/256*joy_y-0.45)))
    robot.bodyPosition((100/256*-joy_rz-20+140, 40+height, 10*xr))
    robot.step()
#    print(joy_z)
gamepad.stop()

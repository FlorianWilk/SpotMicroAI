from kinematics import Kinematic
import numpy as np
import pybullet as p
import pybullet_data
import time
import math
from inputs import devices, get_gamepad
from thinputs import ThreadedInputs  
import matplotlib.pyplot as plt


plot=False
# Target Positions:
start = 0.0
end = 1.0
delta_t = 0.0001
steps = int((end - start) / delta_t)
q_tor = [[0.] * steps, [0.] * steps]
t = [0] * steps

# Dictionary of game controller buttons we want to include.
gamepadInputs = {'ABS_X': 128, 
				'ABS_RZ': 127, 
				'ABS_Y': 126, 
				'ABS_Z': 125, 
				'BTN_SOUTH': 0, 
				'BTN_WEST': 0,
				'BTN_START': 0}


# Initialise the gamepad object using the gamepad inputs Python package
gamepad = ThreadedInputs()
for gamepadInput in gamepadInputs:
	gamepad.append_command(gamepadInput, gamepadInputs[gamepadInput])
gamepad.start()

useMaximalCoordinates = False
useRealTime = 1
fixedTimeStep = 1. / 100
numSolverIterations = 50
t = 0.0
t_end = t + 100
physId = p.connect(p.SHARED_MEMORY)
if (physId < 0):
  p.connect(p.GUI)
angle = 90   

def loadModels():
    #p.setTimeOut(4000000)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(numSolverIterations=200)
    p.setTimeStep(fixedTimeStep)

    orn = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeUid=p.loadURDF("plane.urdf", [0, 0, 0], orn)
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


IDheight = p.addUserDebugParameter("height", -40, 60, 0)
IDroll = p.addUserDebugParameter("roll", -20, 20, 0)

if plot:
    p.stepSimulation()
    figure = plt.figure(figsize=[15, 4.5])
    figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

    ax_pos = figure.add_subplot(141)
    ax_pos.set_title("Joint Position")
    ax_pos.plot(t, q_tor[0], '--r', lw=4, label='Desired q0')
    ax_pos.plot(t, q_tor[1], '--b', lw=4, label='Desired q1')
    ax_pos.set_ylim(-1., 1.)
    ax_pos.legend()
    plt.pause(0.01)

quadruped=loadModels()
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
Lp=np.array([[120,-100,140,1],[120,-100,-140,1],[-120,-100,140,1],[-120,-100,-140,1]])
kin=Kinematic()
p.setRealTimeSimulation(useRealTime)
kp = 0.012
kd = .2
maxForce = 12.5
ref_time = time.time()
for i in range (nJoints):
	p.changeDynamics(quadruped,i,localInertiaDiagonal=[0.000001,0.000001,0.000001])
dirs=[[-1,1,1],[1,1,1],[-1,1,1],[1,1,1]]

p.addUserDebugText("GamePad Mode", [0,0,1.05],textColorRGB=[1,1,1],textSize=1.5)


joy_x=128
joy_y=128
joy_z=128
joy_rz=128
fov, aspect, nearplane, farplane = 90, 1.3, .0111, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
while (1):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(quadruped)
    init_camera_vector = (-1, 0, 0) # z-axis
    init_up_vector = (0, 0, 1) # y-axis
    # Rotated vectors
    rot_matrix = p.getMatrixFromQuaternion(cubeOrn)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    addV=(30,0,0)
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(cubePos+ 0.15 * camera_vector, cubePos + 3 * camera_vector,  up_vector)
    img = p.getCameraImage(320, 200, view_matrix, projection_matrix)
    commandInput, commandValue = gamepad.read()
		# Gamepad button command filter
    if commandInput == 'ABS_X':
        joy_x=commandValue
    if commandInput == 'ABS_Y':
        joy_y=commandValue
    if commandInput == 'ABS_Z':
        joy_z=commandValue
    if commandInput == 'ABS_RZ':
        joy_rz=commandValue
            
    if (useRealTime):
        t = time.time() - ref_time
    else:
        t = t + fixedTimeStep
#    angles=kin.calcIK(Lp,(0.2*math.sin((time.time()-ref_time)*.2),0.3*math.sin((time.time()-ref_time)*.2),0.3*math.cos((time.time()-ref_time)*.6)),#(0,20,0))
#    (0,50+20*math.cos((time.time()-ref_time)*0.3),0))

#    angles=kin.calcIK(Lp,(0,0.3*math.sin((time.time()-ref_time)*.6),0.3*math.cos((time.time()-ref_time)*.2)),#(0,20,0))
#    (0,40,0))

#    angles=kin.calcIK(Lp,(0,-0.2*math.sin((time.time()-ref_time)*2.6),0),#(0,20,0))
#    (30+40*math.sin((time.time()-ref_time)*2.6),40,40*math.cos((time.time()-ref_time)*2.6)))
    height = p.readUserDebugParameter(IDheight)
    roll = p.readUserDebugParameter(IDroll)

    angles=kin.calcIK(Lp,(math.pi/180*roll,0.6/256*joy_x-0.3,-(0.9/256*joy_y-0.45)),#(0,20,0))
    (100/256*-joy_rz-20+100,40+height,60/256*joy_z-30))


    rot=p.getEulerFromQuaternion(cubeOrn)
    (xr,yr,zr)=rot
    if(abs(xr)>math.pi/2 or abs(yr)>math.pi/2):
        p.resetSimulation()
        quadruped=loadModels()        
        continue        
    #print(cubePos,cubeOrn)

    for lx,leg in enumerate(['front_left','front_right','rear_left','rear_right']):
        for px,part in enumerate(['shoulder','leg','foot']):
            j=jointNameToId[leg+"_"+part]
            p.setJointMotorControl2(bodyIndex=quadruped,
                            jointIndex=j,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition= angles[lx][px]*dirs[lx][px],
                            positionGain=kp,
                            velocityGain=kd,
                            force=maxForce)
    #Lp[0][0]+=0.2
    pia=1*math.sin((t/10))
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
gamepad.stop()        
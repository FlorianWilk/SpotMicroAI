"""
SpotMicroAI Simulation
"""

import pybullet_data
import time
import pybullet as p
import math
import numpy as np
from kinematics import Kinematic

class Robot:

    def __init__(self):      

        # Simulation Configuration
        self.useMaximalCoordinates = False
        self.useRealTime = True
        self.fixedTimeStep = 1. / 100
        self.numSolverIterations = 200

        self.init_oritentation=p.getQuaternionFromEuler([0, 0, 90.0])
        self.init_position=[0, 0, 0.3]

        self.reflection=False

        # Parameters for Servos - still wrong
        self.kp = 0.012
        self.kd = .2
        self.maxForce = 12.5

        self.physId = p.connect(p.SHARED_MEMORY)
        if (self.physId < 0):
            p.connect(p.GUI)
        self.angle = 90

        self.oldTextId=0
        self.textId=0
        self.oldDebugInfo=[]
        self.rot=(0,0,0)
        self.pos=(0,0,0)

        if self.reflection:
            p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_reflection, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
        self.IDkp = p.addUserDebugParameter("Kp", 0, 0.05, 0.012) # 0.05
        self.IDkd = p.addUserDebugParameter("Kd", 0, 1, 0.2) # 0.5
        self.IDmaxForce = p.addUserDebugParameter("MaxForce", 0, 50, 12.5)

        p.setRealTimeSimulation(self.useRealTime)

        self.quadruped = self.loadModels()
        self.changeDynamics(self.quadruped)
        self.jointNameToId = self.getJointNames(self.quadruped)

        self.L=140
        self.W=75+5+40

        self.dirs = [[-1, 1, 1], [1, 1, 1], [-1, 1, 1], [1, 1, 1]]
        self.roll=0

        self.Lp = np.array([[120, -100, self.W/2, 1], [120, -100, -self.W/2, 1],
        [-50, -100, self.W/2, 1], [-50, -100, -self.W/2, 1]])

        self.kin = Kinematic()

        p.setRealTimeSimulation(self.useRealTime)
        self.ref_time = time.time()

        # Camera Settings
        fov, aspect, nearplane, farplane = 90, 1.3, .0111, 100
        self.projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)


    def loadModels(self):
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(numSolverIterations=self.numSolverIterations)
        p.setTimeStep(self.fixedTimeStep)

        orn = p.getQuaternionFromEuler([0, 0, 0])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeUid = p.loadURDF("plane_transparent.urdf", [0, 0, 0], orn)
        texUid = p.loadTexture("concrete.png")
        p.changeVisualShape(planeUid, -1, textureUniqueId=texUid)

        stairsUid = p.loadURDF("../urdf/stairs_gen.urdf.xml", [0, -1, 0], orn)

        quadruped = p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", self.init_position,
                            self.init_oritentation,
                            useFixedBase=False,
                            useMaximalCoordinates=self.useMaximalCoordinates,
                            flags=p.URDF_USE_IMPLICIT_CYLINDER)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        return quadruped

    def changeDynamics(self,quadruped):
        nJoints = p.getNumJoints(quadruped)
        for i in range(nJoints):
            p.changeDynamics(quadruped, i, localInertiaDiagonal=[
                            0.000001, 0.000001, 0.000001])

    def getJointNames(self,quadruped):
        nJoints = p.getNumJoints(quadruped)
        jointNameToId = {}

        for i in range(nJoints):
            jointInfo = p.getJointInfo(quadruped, i)
            jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
        return jointNameToId

    
    def addInfoText(self,bodyPos,bodyEuler,linearVel,angularVel):
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
        p.addUserDebugLine([-0.3, 0, 0], [0.3, 0, 0], [0,1,0], parentObjectUniqueId=self.quadruped, parentLinkIndex=1 ),
        p.addUserDebugLine([0, -0.2, 0], [0, 0.2, 0], [0,1,0], parentObjectUniqueId=self.quadruped, parentLinkIndex=1 ),

        if len(self.oldDebugInfo)>0:
            for x in self.oldDebugInfo:
                p.removeUserDebugItem(x)
        self.oldDebugInfo=newDebugInfo


    def handleCamera(self,cubePos, cubeOrn):

        # Look forward/up
        init_camera_vector = (-1, 0, 0)
        init_up_vector = (0, 0, 1)

        # rotate camera on y by 25degrees
        orn = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, -math.pi/180*25,0]))

        # Rotate vectors with body
        orn = np.array(orn).reshape(3, 3)
        rot_matrix = p.getMatrixFromQuaternion(cubeOrn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        camera_vector = rot_matrix.dot(orn.dot(init_camera_vector))
        up_vector = rot_matrix.dot(orn.dot(init_up_vector))

        # Camera origin is right at the end of the head watching forward (Body-Coordinates)
        view_matrix = p.computeViewMatrix(
            cubePos + 0.17 * camera_vector, cubePos + 3 * camera_vector,  up_vector)
        img = p.getCameraImage(320, 200, view_matrix, self.projection_matrix)
        #TODO: Use the camera image

    def checkSimulationReset(self,bodyOrn):
        (xr, yr, _) = p.getEulerFromQuaternion(bodyOrn)
        
        # If our Body rotated more than pi/3: reset
        if(abs(xr) > math.pi/3 or abs(yr) > math.pi/3):
            p.resetBasePositionAndOrientation(self.quadruped, self.init_position,self.init_oritentation)
            p.resetBaseVelocity(self.quadruped, [0, 0, 0], [0, 0, 0])
            return True
        return False

    def bodyRotation(self,rot):
        self.rot=rot

    def bodyPosition(self,pos):
        self.pos=pos

    def feetPosition(self,Lp):
        self.Lp=Lp
  
    def getIMU(self):
        _, bodyOrn = p.getBasePositionAndOrientation(self.quadruped)
        linearVel, angularVel = p.getBaseVelocity(self.quadruped)
        return bodyOrn,linearVel,angularVel

    def step(self):
        quadruped=self.quadruped
        bodyPos, bodyOrn = p.getBasePositionAndOrientation(quadruped)
        linearVel, angularVel = p.getBaseVelocity(quadruped)
        bodyEuler=p.getEulerFromQuaternion(bodyOrn)
        kp=p.readUserDebugParameter(self.IDkp)
        kd=p.readUserDebugParameter(self.IDkd)
        maxForce=p.readUserDebugParameter(self.IDmaxForce)

        self.handleCamera(bodyPos, bodyOrn)
        self.addInfoText(bodyPos,bodyEuler,linearVel,angularVel)

        if self.checkSimulationReset(bodyOrn):
            return False

        # Calculate Angles with the input of FeetPos,BodyRotation and BodyPosition
        angles = self.kin.calcIK(self.Lp, self.rot, self.pos)

        for lx, leg in enumerate(['front_left', 'front_right', 'rear_left', 'rear_right']):
            for px, part in enumerate(['shoulder', 'leg', 'foot']):
                j = self.jointNameToId[leg+"_"+part]
                p.setJointMotorControl2(bodyIndex=quadruped,
                                        jointIndex=j,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=angles[lx][px]*self.dirs[lx][px],
                                        positionGain=kp,
                                        velocityGain=kd,
                                        force=maxForce)        
        # let the Simulator do the rest    
        if (self.useRealTime):
            t = time.time() - self.ref_time
        else:
            t = t + self.fixedTimeStep
        if (self.useRealTime == False):
            p.stepSimulation()
            time.sleep(self.fixedTimeStep)
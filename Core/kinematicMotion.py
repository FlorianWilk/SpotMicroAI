import time
import numpy as np
import math
from kinematics import Kinematic

class KinematicLegMotion:

    def __init__(self,LLp):
        self.rtime=time.time()
        self.running=False
        self.LLp=LLp

    def moveTo(self,newLLp,rtime,func=None):
        if self.running:
            # TODO: Queue the Requests
            print("Movement already running, please try again later.")
            return False
        self.startTime=time.time()
        self.startLLp=self.LLp
        self.func=func
        self.targetLLp=newLLp
        self.endTime=time.time()+rtime/1000
        self.running=True
        return True
    
    def update(self):
        diff=time.time()-self.startTime
        ldiff=self.targetLLp-self.startLLp
        tdiff=self.endTime-self.startTime
        ldiff/(tdiff*diff)
        p=1/tdiff*diff

        if time.time()>self.endTime and self.running:
            self.running=False
            p=1
        self.LLp=self.startLLp+ldiff*p
        if self.func:
            self.LLp=self.func(p,self.LLp)

    def step(self):
        if self.running:
            self.update()
        return self.LLp

class KinematicMotion:

    def __init__(self,Lp):
        self.Lp=Lp
        self.legs=[KinematicLegMotion(Lp[x]) for x in range(4)]

    def moveLegsTo(self,newLp,rtime):
        [self.legs[x].moveTo(newLp[x],rtime) for x in range(4)]

    def moveLegTo(self,leg,newLLp,rtime,func=None):
        return self.legs[leg].moveTo(newLLp,rtime,func)

    def step(self):
        return [x.step() for x in self.legs]


"""
This class will define the trotting-gait function
A complete cycle is tone in Tt
Each leg has the following "states"
0 - wait on ground for t0
1 - move on ground for steplength Sl for t1
2 - wait on ground for t2
3 - lift leg by Sh and Sl for t3 back to 0
"""
class TrottingGait:
    
    def __init__(self):
        self.maxSl=2
        self.bodyPos=(0,100,0)
        self.bodyRot=(0,0,0)
        self.t0=0
        self.t1=500
        self.t2=00
        self.t3=300
        self.Sl=-150 
        self.Sh=40

    def calcLeg(self,t,x,y,z):
        startLp=np.array([x-self.Sl/2.0,y,z,1])
        endY=0 #-0.8 # delta y to jump a bit before lifting legs
        endLp=np.array([x+self.Sl/2,y+endY,z,1])
        if(t<self.t0):
            return startLp
        elif(t<self.t0+self.t1):
            td=t-self.t0
#            print(td)
            tp=1/(self.t1/td)
            diffLp=endLp-startLp
            curLp=startLp+diffLp*tp
            return curLp
        elif(t<self.t0+self.t1+self.t2):
            return endLp
        elif(t<self.t0+self.t1+self.t2+self.t3):
            td=t-(self.t0+self.t1+self.t2)
            tp=1/(self.t3/td)
            diffLp=startLp-endLp
            curLp=endLp+diffLp*tp
            curLp[1]+=self.Sh*math.sin(math.pi*tp)
            return curLp

    def positions(self,t):
        spurWidth=80 # Todo: ugly and wrong
        Tt=self.t0+self.t1+self.t2+self.t3
        rd=-2 # rear delta
        td=(t*1000)%Tt
        t2=(t*1000-Tt/2)%Tt
        rtd=(t*1000-rd)%Tt # rear time delta
        rt2=(t*1000-Tt/2-rd)%Tt
        lz=spurWidth
        rz=-spurWidth
        #td=0
        #t2=0
        Fx=120
        Fy=-100
        Rx=-100
        Ry=-100
        r=np.array([self.calcLeg(td,Fx,Fy,lz),self.calcLeg(t2,Fx,Fy,rz),self.calcLeg(rt2,Rx,Ry,lz),self.calcLeg(rtd,Rx,Ry,rz)])
        #print(r)
        return r
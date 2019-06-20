import numpy as np
import time
import math

class TrottingGait:
    
    def __init__(self):
        self.maxSl=2
        self.bodyPos=(0,100,0)
        self.bodyRot=(0,0,0)
        self.t0=0 # senseless i guess
        self.t1=510
        self.t2=00
        self.t3=185
        self.Sl=-124
        self.Sw=0
        self.Sh=60
        self.Sa=0
        self.Spf=87
        self.Spr=77
        self.Rc=[-50,0,0,1] # rotation center


    """
    calculates the Lp - LegPosition for the configured gait for time t and original Lp of x,y,z
    """
    def calcLeg(self,t,x,y,z):
        startLp=np.array([x-self.Sl/2.0,y,z-self.Sw,1])
        endY=0 #-0.8 # delta y to jump a bit before lifting legs
        endLp=np.array([x+self.Sl/2,y+endY,z+self.Sw,1])
        
        if(t<self.t0): # TODO: remove t0 and t2 - not practical
            return startLp
        elif(t<self.t0+self.t1): # drag foot over ground

            td=t-self.t0
            tp=1/(self.t1/td)
            diffLp=endLp-startLp
            curLp=startLp+diffLp*tp
            psi=-((math.pi/180*self.Sa)/2)+(math.pi/180*self.Sa)*tp
            Ry = np.array([[np.cos(psi),0,np.sin(psi),0],
                    [0,1,0,0],
                    [-np.sin(psi),0,np.cos(psi),0],[0,0,0,1]])
            #Tlm = np.array([[0,0,0,-self.Rc[0]],[0,0,0,-self.Rc[1]],[0,0,0,-self.Rc[2]],[0,0,0,0]])
            curLp=Ry.dot(curLp)
            return curLp
        elif(t<self.t0+self.t1+self.t2):
            return endLp
        elif(t<self.t0+self.t1+self.t2+self.t3): # Lift foot
            td=t-(self.t0+self.t1+self.t2)
            tp=1/(self.t3/td)
            diffLp=startLp-endLp
            curLp=endLp+diffLp*tp
            curLp[1]+=self.Sh*math.sin(math.pi*tp)
            return curLp
    def stepLength(self,len):
        self.Sl=len

    def positions(self,t):
        Tt=(self.t0+self.t1+self.t2+self.t3)
        Tt2=Tt/2
        rd=0 # rear delta - unused - maybe stupid
        td=(t*1000)%Tt
        t2=(t*1000-Tt2)%Tt
        rtd=(t*1000-rd)%Tt # rear time delta
        rt2=(t*1000-Tt2-rd)%Tt
        Fy=-100
        Ry=-100
        r=np.array([self.calcLeg(td,Fx,Fy,spf),self.calcLeg(t2,Fx,Fy,-spf),self.calcLeg(rt2,Rx,Ry,spr),self.calcLeg(rtd,Rx,Ry,-spr)])
        #print(r)
        return r
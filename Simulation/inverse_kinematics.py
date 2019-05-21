"""
See https://tote.readthedocs.io/en/latest/ik.html
https://www.ijstr.org/final-print/sep2017/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf
"""
import numpy as np
import math

class IK(object):
    def __init__(self):


        # Dimensions of the Body-IK

        self._l=18.5
        self._w=16

        # Length of the Leg-Segments

        self._l1=2.5
        self._l2=1.5
        self._l3=11.8
        self._l4=12.3

        self._default_foot_position=np.matrix([2.5,self._l2+self._l3+self._l4,0,1])

    def calc_leg_ik(self,x,y,z):
        
        omega = 0.0 # Body xrot
        phi = 1.3 # Body YRot
        psi = 0 # Body ZRot

        L = self._l
        W = self._w

        xm = 0
        ym = 0
        zm = 0
        
        # Transformation Matrix for Forward Kinematics

        Rx = np.matrix([
            [1, 0, 0, 0], 
            [0, np.cos(omega), -np.sin(omega), 0],
            [0,np.sin(omega),np.cos(omega),0],
            [0,0,0,1]])

        Ry = np.matrix([
            [np.cos(phi),0, np.sin(phi), 0], 
            [0, 1, 0, 0],
            [-np.sin(omega),0, np.cos(omega),0],
            [0,0,0,1]])

        Rz = np.matrix([
            [np.cos(psi),-np.sin(psi), 0,0], 
            [np.sin(psi),np.cos(psi),0,0],
            [0,0,1,0],
            [0,0,0,1]])
        
        Rxyz=Rx*Ry*Rz

        T = np.matrix([[1,0,0,xm],[0,1,0,ym],[0,0,1,zm],[0,0,0,1]])

        Tm = Rxyz*T

        Trb = Tm * np.matrix([
            [np.cos(math.pi/2),0,np.sin(math.pi/2),-L/2],
            [0,1,0,0],
            [-np.sin(math.pi/2),0,np.cos(math.pi/2),W/2],
            [0,0,0,1]])

        Trf = Tm * np.matrix([
            [np.cos(math.pi/2),0,np.sin(math.pi/2),L/2],
            [0,1,0,0],
            [-np.sin(math.pi/2),0,np.cos(math.pi/2),W/2],
            [0,0,0,1]])

        Tlf = Tm * np.matrix([
            [np.cos(math.pi/2),0,np.sin(math.pi/2),L/2],
            [0,1,0,0],
            [-np.sin(math.pi/2),0,np.cos(math.pi/2),-W/2],
            [0,0,0,1]])

        Tlb = Tm * np.matrix([
            [np.cos(math.pi/2),0,np.sin(math.pi/2),-L/2],
            [0,1,0,0],
            [-np.sin(math.pi/2),0,np.cos(math.pi/2),-W/2],
            [0,0,0,1]])

        P = Tlb*np.matrix([x,y,z,1]).T

        print P

        l1=self._l1
        l2=self._l2
        l3=self._l3
        l4=self._l4

        # Shoulder

        alpha1=-math.atan2(-y,x) 
        E=math.sqrt(x**2+y**2-l1**2)
        beta1=math.atan2(E,-l1)
        theta1=alpha1-beta1

        # Foot

        F=E-l2
        D=(F**2+z**2-l3**2-l4**2)/(2*l3*l4)
        theta3=math.atan2(math.sqrt(1-D**2),D)

        # Leg

        theta2=math.atan2(z,F)-math.atan2(l3*math.sin(theta3),l2+l3*math.cos(theta3))

        print("t1 {} t2 {} t3 {}".format(theta1/(math.pi/180),theta2/(math.pi/180),theta3/(math.pi/180)))
        return (theta1,theta2,theta3)

if __name__ == "__main__":
    IK().calc_leg_ik(2.5,18,0.0)

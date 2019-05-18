"""
See https://tote.readthedocs.io/en/latest/ik.html
https://www.ijstr.org/final-print/sep2017/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf
"""

import math

class IK(object):
    def __init__(self):
        self._shoulder_width = 2.5
        self._leg_length=11.8
        self._foot_length=12.3

    def calc(self,x,y,z):
        """ Notes:
        atan2 angle of diagonale in rectangle
        acos alpha from triangle for which we known all length

        Assuming X goes from front to back, Y left right, Z up down

        alpha is the shoulder angle
        beta the leg angle
        gamma the foot angle
        """

        alpha = math.atan2(y-self._shoulder_width,z) 
        
        f=math.sqrt(z**2+y**2)
        d=math.sqrt(f**2+x**2)
        beta1=math.atan2(z,f)
        beta2=math.acos((self._leg_length**2+d**2-self._foot_length**2)/(2*self._leg_length*d))
        beta=beta1+beta2

        gamma = math.acos((self._leg_length**2+self._foot_length**2-d**2)/(2*self._leg_length*self._foot_length))
        print("alpha {} beta {} gamma {}".format(alpha/(math.pi/180),beta/(math.pi/180),gamma/(math.pi/180)))

if __name__ == "__main__":
    IK().calc(11.8,0,12.3)
    print("OK")
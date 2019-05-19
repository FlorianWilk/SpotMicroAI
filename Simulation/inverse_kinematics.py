"""
See https://tote.readthedocs.io/en/latest/ik.html
https://www.ijstr.org/final-print/sep2017/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf
"""

import math

class IK(object):
    def __init__(self):
        self._l1=2.5
        self._l2=1.5
        self._l3=11.8
        self._l4=12.3

    def calc(self,x,y,z):
        """ Notes:
        atan2 angle of diagonale in rectangle
        acos alpha from triangle for which we known all length

        Assuming Z goes from front to back, X left right, Z up down

        alpha is the shoulder angle
        beta the leg angle
        gamma the foot angle
        """

        l1=self._l1
        l2=self._l2
        l3=self._l3
        l4=self._l4

        # Shoulder

        alpha1=-math.atan2(-y,x) 
        E=math.sqrt(x**2+y**2-l1**2)
        beta1=math.atan2(E,-l1)
        omega1=alpha1-beta1

        # Foot

        F=E-l2
        D=(F**2+z**2-l3**2-l4**2)/(2*l3*l4)
        omega3=math.atan2(math.sqrt(1-D**2),D)

        # Leg

        omega2=math.atan2(z,F)-math.atan2(l3*math.sin(omega3),l2+l3*math.cos(omega3))

        print("o1 {} o2 {} o3 {}".format(omega1/(math.pi/180),omega2/(math.pi/180),omega3/(math.pi/180)))

if __name__ == "__main__":
    IK().calc(-2.5,18,0.3)
    print("OK")
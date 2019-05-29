import time
import math
import numpy as np
from kinematics import Kinematic
from servos import Servos
class RobotRunner:

    def __init__(self):
        print("Initialising")
        self.servos=Servos()
        self.Kinematic=Kinematic()

        self.servos.setServoPositions([0 for _ in range(0,12)])

    def calibrate(self):
        print("Running")
        servo=int(input("Enter Servo to calibrate (0-12)"))
        print("Current Offset is {}".format(self.servos.getServoOffset(servo)))
        offset=int(input("Enter new Offset:"))
        self.servos.setServoOffset(servo,offset)


RobotRunner().calibrate()
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

    def run(self):
        print("Running")

RobotRunner().run()
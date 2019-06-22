#!/usr/bin/env python
"""
SpotMicroAI Bootscript
"""

from display import RobotDisplay
from gyro import Gyro
#from servos import Servos
import atexit
import sys
import signal
import time
from kinematics import Kinematic

class RobotBoot():

    def __init__(self):
        self.display=None
        print("Booting SpotMicroAI")
        self.display=RobotDisplay()
#        self.servos=Servos()
        self.Kinematic=Kinematic()

    def exitHandler(self):
        print("Exiting SpotMicroAI")
        if not self.display==None:
            self.display.bye()
        sys.exit()

    def run(self):
        while True:
            self.display.run()
            time.sleep(0.1)

if __name__ == "__main__":
    try:
       boot=RobotBoot()
#       atexit.register(boot.exitHandler)
       signal.signal(signal.SIGTERM, boot.exitHandler)
       boot.run()
    except KeyboardInterrupt:
        pass



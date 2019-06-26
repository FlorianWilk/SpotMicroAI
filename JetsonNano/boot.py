#!/usr/bin/env python
"""
SpotMicroAI Bootscript
"""

from display import RobotDisplay
from gyro import Gyro
from servos import Servos
import atexit
import sys
import signal
import time
from kinematics import Kinematic
from network import Network

class RobotBoot():

    def __init__(self):
        self.display=None
        print("Booting SpotMicroAI")
        self.display=RobotDisplay()
        self.gyro=Gyro()
#        self.servos=Servos()
        self.Kinematic=Kinematic()
        self.network=Network()

    def exitHandler(self):
        print("Exiting SpotMicroAI")
        if not self.display==None:
            self.display.bye()
        sys.exit()

    def run(self):
        while True:
            (x,y)=self.gyro.read()
            self.display.setAngles(x,y)
            self.display.setNetworkState(self.network.result())
            self.display.run()
            time.sleep(0.02)

    def cleanup(self):
        self.network.cleanup()

if __name__ == "__main__":
    try:
        boot=RobotBoot()
#       atexit.register(boot.exitHandler)
        signal.signal(signal.SIGTERM, boot.exitHandler)
        boot.run()
    except:
        boot.cleanup()


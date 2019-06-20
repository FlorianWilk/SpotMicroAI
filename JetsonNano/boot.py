#!/usr/bin/env python
"""
SpotMicroAI Bootscript
"""

from display import RobotDisplay
from servos import Servos
import atexit

class RobotBoot():

    def __init__(self):
        self.display=None
        atexit.register(self.exitHandler)
        print("Booting SpotMicroAI")
        self.display=RobotDisplay()
        self.servos=Servos()

    def exitHandler(self):
        print("Exiting SpotMicroAI")
        if not self.display==None:
            self.display.bye()

    def run(self):
        while True:
            self.display.run()

if __name__ == "__main__":
    try:
       boot=RobotBoot()
       boot.run()
    except KeyboardInterrupt:
        pass



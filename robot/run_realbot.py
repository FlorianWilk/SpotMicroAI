"""
SpotMicroAI - RealBot 
This is the main code for the physical Robot
"""

import numpy as np
import time
import math
import core.kinematics as ck
import realbot.servos as sr

kinematics = ck.Kinematics()
s1=sr.Servo(1,1)
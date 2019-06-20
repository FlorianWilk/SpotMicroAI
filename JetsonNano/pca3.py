# This example moves a servo its full range (180 degrees by default) and then back.

from board import SCL_1, SDA_1
import busio

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
import time

# This example also relies on the Adafruit motor library available here:
# https://github.com/adafruit/Adafruit_CircuitPython_Motor
from adafruit_motor import servo

i2c = busio.I2C(SCL_1, SDA_1)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2480)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2400)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2600)

# The pulse range is 1000 - 2000 by default.
servo7 = servo.Servo(pca.channels[7])
while True:
    for i in range(180):
        servo7.angle = i
    for i in range(180):
        servo7.angle = 180 - i
        time.sleep(0.1)
pca.deinit()

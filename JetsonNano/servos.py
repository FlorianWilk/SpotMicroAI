"""
Class to Control the Servos for all Legs
"""
import Adafruit_PCA9685
import board
import busio

class Servo:
    def __init__(self,direction,servoId):
        self.direction=direction
        self.offset=0
        self.min=0
        self.max=180
        self.servoId=servoId
        self.lastValue=0

    def setValue(self,value):
        self.lastValue=value
        self.realValue=self.offset+self.direction*value
        # I2C set calculated Value 
    
    def getLastValue(self):
        return self.lastValue

    def setOffset(self,offset):
        self.offset=offset
        self.setValue(self.getLastValue())

class AcceleratedServo(Servo):
    def __init__(self,direction,servoId):
        Servo.__init__(self,direction,servoId)

        # random values for now
        self.kp=0.2
        self.kd=0.2
        self.max_acceleration=14

    def moveTo(self,value,inTime):
        print("Moving servo to {} in {}ms".format(value,inTime))

class Servos:

    def __init__(self):
        print("Servos init")

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = Adafruit_PCA9685.PCA9685(i2c)
        #self.pca = Adafruit_PCA9685.PCA9685(address=0x40)
        
        # Front_left,Front_right,Back_left,Back_right
        # each leg has shoulder, leg, foot
        directions=[-1,1,1,1,1,1,-1,1,1,1,1,1]
        self.leg_servos=[AcceleratedServo(directions[x],1) for x in range(0,12)]

    def setServoPositions(self,positions):
        [self.leg_servos[x].moveTo(positions[x],100) for x in range(0,12)]
    
    def getServoOffset(self,servo):
        return self.leg_servos[servo].offset

    def setServoOffset(self,servo,offset):
        self.leg_servos[servo].setOffset(offset)

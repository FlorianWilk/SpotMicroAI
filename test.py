from adafruit_servokit import ServoKit
import time
class Dog(object):
    def __init__(self):
        self.FL = Leg("F", "L", [0, 5, 4], kit.servo, offsets= [0, 0, -35])
        self.FR = Leg("F", "R", [1, 3, 2], kit.servo)
        self.BL = Leg("B", "L", [13, 11, 8], kit.servo)
        self.BR = Leg("B", "R", [12, 10, 9], kit.servo)
        self.legs = {"FL" : self.FL, "FR" : self.FR, "BL" : self.BL, "BR" : self.BR}
    def __del__(self):
        self.allOff()
        # for _, leg in self.legs.values():
        #     leg.allOff()
    def allOff(self):
    for x in range(len(kit.servo)):
        kit.servo[x].angle = None


        
class Leg(object):
    # self.pos.wrist = None
    # self.pos.elbow = None
    # self.pos.shoulder = None
    def __init__(self, end, side, servos, servoController, offsets = [0, 0, 0]):
        self.offsets = offsets
        self.servoController = servoController
        self.side = side #left or right
        self.end = end #front or back
        self.wrist = {
            "servo" : servos[0],
            "currentPos" : None,
            "offset" : offsets[0]}
        self.elbow = {
            "servo" : servos[1],
            "currentPos" : None,
            "offset" : offsets[1]}
        self.shoulder = {
            "servo" : servos[2],
            "currentPos" : None,
            "offset" : offsets[2]}
        self.position = {
            "UP" : [180, 0, 90],
            "DOWN" : [0, 180, 90],
            "DEFAULT": [90, 90, 90],
            "A": [180, 0, 90],
            "B": [90, 80, 90],
            "C": [90, 90, 40],
            "D": [90, 90, 140]
        }
    def _move(self, joint, angle):
        a = self.offset(joint, angle)
        self.servoController[joint["servo"]].angle = a
    
    def offset(self, joint, angle):
        print(joint, angle)
        if self.side == "L":
            angle = 180 - angle
        if joint["servo"] == self.shoulder["servo"] and self.end == "F":
            angle = 180 - angle
            

        a = angle - joint["offset"]
        return a
        

    def move(self, position):
        self._move(self.wrist, position[0])
        self._move(self.elbow, position[1])
        self._move(self.shoulder, position[2])
        # self.pos.wrist = position[0]
        # self.pos.elbow = position[1]
        # self.pos.shoulder = position[2]

    def up(self):
        self.move(self.position["UP"])

    def preset(self, pos):
        self.move(self.position[pos])

    def down(self):
        self.move(self.position["DOWN"])
            
    def neutral(self):
        self.move(self.position["DEFAULT"])



def test(s):
    restTime = 1.5
    move(s, 0)
    time.sleep(restTime)
    move(s,180)
    time.sleep(restTime)
    move(s, 90)
    time.sleep(restTime)
    move(s, None)

def testAll():
    for x in range(len(kit.servo)):
        print(f"testing {x}")
        test(x)
    
def testArm(leg):
    move(leg[0], 180)
    move(leg[1], 0)
    move(leg[2], 90)
    time.sleep(3)
    move(leg[0], 0)
    move(leg[1], 180)
    move(leg[2], 90)
    time.sleep(3)
    allOff()

def moveLeg(leg, location):
    move(leg[0], location[0])
    move(leg[1], location[1])
    move(leg[2], location[2])

def armTester():
    while(True):
        numIn = int(input("Please enter the servo number to test:\n"))
        if numIn == "q":
            print("ending session")
            break
        elif numIn not in range(len(kit.servo)):
            print("error, invalid number")
            break
        test(numIn)



# while(True):
#     numIn = int(input("Please enter the arm number to test:\n"))
#     testArm(FrontRightArm)


def main(kit):
    #Arm = [forelimb, midlimb, shoulder]
    FL = Leg("F", "L", [0, 5, 4], kit.servo, offsets= [0, 0, -35])
    FR = Leg("F", "R", [1, 3, 2], kit.servo)
    BL = Leg("B", "L", [13, 11, 8], kit.servo)
    BR = Leg("B", "R", [12, 10, 9], kit.servo)

    allLegs = [FL, FR, BL, BR]

    while(True):
        for x in ["C", "D"]:
            i = input('press enter to continue...')
            if i =="q":
                return None
            for leg in allLegs:
                leg.preset(x)



# print("ending session")
# kit.servo[0].angle 
if __name__ == "__main__":
    print("here we go!")
    kit = ServoKit(channels=16)
    print("servos initialized")
    main(kit)
    allOff()
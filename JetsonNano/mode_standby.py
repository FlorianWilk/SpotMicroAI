from modes import AbstractMode
from servos import Servos

class ModeStandby(AbstractMode):

    def __init__(self,servos):
        AbstractMode.__init__(self,servos)

    def init(self):
        [self.servos.angle(v,0) for v in range(0,11)]

    def update(self):
        pass
from servos import Servos

class AbstractMode():

    def __init__(self,servos):
        self.servos=servos

    def init(self):
        pass

    def update(self):
        pass
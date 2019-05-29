import time

class KinematicLegMotion:

    def __init__(self,LLp):
        self.rtime=time.time()
        self.running=False
        self.LLp=LLp

    def moveTo(self,newLLp,rtime):
        if self.running:
            # TODO: Queue the Requests
            print("Movement already running, please try again later.")
            return
        self.startTime=time.time()
        self.startLLp=self.LLp
        self.targetLLp=newLLp
        self.endTime=time.time()+rtime/1000
        self.running=True
    
    def update(self):
        if time.time()>self.endTime and self.running:
            self.running=False
            return
        diff=time.time()-self.startTime
        ldiff=self.targetLLp-self.startLLp
        tdiff=self.endTime-self.startTime
        ldiff/(tdiff*diff)
        p=1/tdiff*diff
        self.LLp=self.startLLp+ldiff*p

    def step(self):
        if self.running:
            self.update()
        return self.LLp

class KinematicMotion:

    def __init__(self,Lp):
        self.Lp=Lp
        self.legs=[KinematicLegMotion(Lp[x]) for x in range(4)]

    def moveLegsTo(self,newLp,rtime):
        [self.legs[x].moveTo(newLp[x],rtime) for x in range(4)]

    def moveLegTo(self,leg,newLLp,rtime):
        self.legs[leg].moveTo(newLLp,rtime)

    def step(self):
        return [self.legs[x].step() for x in range(4)]


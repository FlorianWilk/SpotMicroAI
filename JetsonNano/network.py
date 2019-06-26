"""
SpotMicroAI Network Utils
"""
from threading import Thread
import time
import socket

class Network():

    def __init__(self):
        self.running=True  
        self.t = Thread(target=self.checker, args=(0,))
        self.t.start()  
        self.lastCheckResult=False
        self.lastCheck=0

    def result(self):
        return self.lastCheckResult

    def check(self):
        self.lastCheck=time.time()
        try:
            socket.setdefaulttimeout(1)
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
            self.lastCheckResult=True
            return True
        except Exception as ex:
            self.lastCheckResult=False
            return False        

    def checker(self,i):
        while self.running:
            self.check()
            time.sleep(10)

    def cleanup(self):
        self.running=False
        self.t.join()

if __name__ == "__main__":
    try:
        net = Network()
        while True:
            time.sleep(10)
    except:
        net.cleanup()
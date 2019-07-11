import os
import sys
import time
from datetime import datetime
from demo_opts import get_device
from luma.core.render import canvas
from PIL import ImageFont
import math
try:
    import psutil
except ImportError:
    print("The psutil library was not found. Run 'sudo -H pip install psutil' to install it.")
    sys.exit()

#TODO: Refactoring + Cleanups

def bytes2human(n):
    """
    >>> bytes2human(10000)
    '9K'
    >>> bytes2human(100001221)
    '95M'
    """
    symbols = ('K', 'M', 'G', 'T', 'P', 'E', 'Z', 'Y')
    prefix = {}
    for i, s in enumerate(symbols):
        prefix[s] = 1 << (i + 1) * 10
    for s in reversed(symbols):
        if n >= prefix[s]:
            value = int(float(n) / prefix[s])
            return '%s%s' % (value, s)
    return "%sB" % n

lastCpuUsageTime=0
lastCpuUsage=""

def cpu_usage():
    global lastCpuUsageTime,lastCpuUsage
    if(time.time()-lastCpuUsageTime<3):
        return lastCpuUsage
    # load average, uptime
    uptime = datetime.now() - datetime.fromtimestamp(psutil.boot_time())
    av1, av2, av3 = os.getloadavg()
    lastCpuUsage="Ld:%.1f %.1f %.1f Up: %s" \
        % (av1, av2, av3, str(uptime).split('.')[0])
    lastCpuUsageTime=time.time()
    return lastCpuUsage

lastMemUsageTime=0
lastMemUsage=""

def mem_usage():
    global lastMemUsageTime,lastMemUsage
    if(time.time()-lastMemUsageTime<5):
        return lastMemUsage
    usage = psutil.virtual_memory()
    lastMemUsageTime=time.time()
    lastMemUsage="Mem: %s %.0f%%" \
        % (bytes2human(usage.used), 100 - usage.percent)
    return lastMemUsage

lastDiskUsageTime=0
lastDiskUsage=""

def disk_usage(dir):
    global lastDiskUsageTime,lastDiskUsage
    if(time.time()-lastDiskUsageTime<10.0):
        return lastDiskUsage
    usage = psutil.disk_usage(dir)
    lastDiskUsage="SD: %s" \
        % (bytes2human(usage.used))#, usage.percent)
    lastDiskUsageTime=time.time()
    return lastDiskUsage


def network(iface):
    stat = psutil.net_io_counters(pernic=True)[iface]
    return "%s: Tx%s, Rx%s" % \
           (iface, bytes2human(stat.bytes_sent), bytes2human(stat.bytes_recv))

class RobotDisplay():
    
    def __init__(self):
        self.angles=(0,0)
        self.device=get_device()
        self.font_fa=self.make_font("fa-regular-400.ttf", self.device.height - 10)
        self.font2=self.make_font("C&C Red Alert [INET].ttf",12)

        # Show the Smiley on Boot
        self.displayIcon("\uf599")
        self.networkState=False
        time.sleep(2)


    def make_font(self,name, size):
        font_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), 'fonts', name))
        print("looking up font {}".format(font_path))
        return ImageFont.truetype(font_path, size)

    def setAngles(self,x,y):
        self.angles=(x,y)

    def setNetworkState(self,state):
        self.networkState=state

    def drawRot(self,draw,x,val):
        x1=18*math.cos(math.pi/180*val)
        y1=18*math.sin(math.pi/180*val)
        d=10
        e=3
        draw.ellipse((4+x,29,38+x,63), outline="white")
        #draw.line((x+21-x1,46-y1,x+22+x1,47+y1), width=2,fill="white")
        draw.chord((4+x+e,29+e,38+x-e,63-e),0+val,180+val,fill="white",outline="white")
        draw.ellipse((21-d+x,46-d,21+d+x,46+d), fill="black")
        tx="{:0.0f}".format(val)
        w, h = draw.textsize(text=tx, font=self.font2)
        left = (22+x - w/2)
        top = 40
        draw.text((left,top),tx,font=self.font2,fill="white")
        

    def stats(self,device):
        x,y=self.angles
        with canvas(device) as draw:
            draw.text((0, 0), cpu_usage(), font=self.font2, fill="white")
            if device.height >= 32:
                draw.text((0, 13), mem_usage(), font=self.font2, fill="white")

            if device.height >= 64:
                draw.text((78, 13), disk_usage('/'), font=self.font2, fill="white")
                self.drawRot(draw,-4,x)
                self.drawRot(draw,36,y)
            # TODO: Add more states like NC network connection or LN local network
            state="NN"  # no network
            if self.networkState:
                state="IC" # internet connection
            draw.text((100, 13),state, font=self.font2, fill="white")

    def run(self):
        self.stats(self.device)

    def bye(self):
        self.displayIcon("\uf186")
        time.sleep(2)

    def displayIcon(self,code):
        with canvas(self.device) as draw:
            w, h = draw.textsize(text=code, font=self.font_fa)
            left = (self.device.width - w) / 2
            top = (self.device.height - h) / 1
            draw.text((left, top), text=code, font=self.font_fa, fill="white")
        time.sleep(2)


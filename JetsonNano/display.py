import os
import sys
import time
from datetime import datetime
from demo_opts import get_device
from luma.core.render import canvas
from PIL import ImageFont
try:
    import psutil
except ImportError:
    print("The psutil library was not found. Run 'sudo -H pip install psutil' to install it.")
    sys.exit()


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

def cpu_usage():
    # load average, uptime
    uptime = datetime.now() - datetime.fromtimestamp(psutil.boot_time())
    av1, av2, av3 = os.getloadavg()
    return "Ld:%.1f %.1f %.1f Up: %s" \
        % (av1, av2, av3, str(uptime).split('.')[0])


def mem_usage():
    usage = psutil.virtual_memory()
    return "Mem: %s %.0f%%" \
        % (bytes2human(usage.used), 100 - usage.percent)


def disk_usage(dir):
    usage = psutil.disk_usage(dir)
    return "SD:  %s %.0f%%" \
        % (bytes2human(usage.used), usage.percent)


def network(iface):
    stat = psutil.net_io_counters(pernic=True)[iface]
    return "%s: Tx%s, Rx%s" % \
           (iface, bytes2human(stat.bytes_sent), bytes2human(stat.bytes_recv))

class RobotDisplay():
    
    def __init__(self):
        self.device=get_device()
        self.font_fa=self.make_font("fa-regular-400.ttf", self.device.height - 10)
        self.font2=self.make_font("C&C Red Alert [INET].ttf",12)
        self.displayIcon("\uf599")
        time.sleep(2)

    def make_font(self,name, size):
        font_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), 'fonts', name))
        print("looking up font {}".format(font_path))
        return ImageFont.truetype(font_path, size)

    def stats(self,device):

        with canvas(device) as draw:
            draw.text((0, 0), cpu_usage(), font=self.font2, fill="white")
            if device.height >= 32:
                draw.text((0, 14), mem_usage(), font=self.font2, fill="white")

            if device.height >= 64:
                draw.text((0, 26), disk_usage('/'), font=self.font2, fill="white")
                try:
                    draw.text((0, 38), network('wlan0'), font=self.font2, fill="white")
                except KeyError:
                    # no wifi enabled/available
                    pass

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


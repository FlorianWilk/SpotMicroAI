#!/usr/bin/python
import smbus
import math

class Gyro():
    def __init__(self):
        # Register
        self.power_mgmt_1 = 0x6b
        self.power_mgmt_2 = 0x6c
        self.bus = smbus.SMBus(0) 
        self.address = 0x68       
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
 
    def read_byte(self,reg):
        return self.bus.read_byte_data(self.address, reg)
 
    def read_word(self,reg):
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg+1)
        value = (h << 8) + l
        return value
 
    def read_word_2c(self,reg):
        val = self.read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
 
    def dist(self,a,b):
        return math.sqrt((a*a)+(b*b))
 
    def get_y_rotation(self,x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        return -math.degrees(radians)
 
    def get_x_rotation(self,x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)
 
    def read(self):
        #gyroskop_xout = self.read_word_2c(0x43)
        #gyroskop_yout = self.read_word_2c(0x45)
        #gyroskop_zout = self.read_word_2c(0x47)
        
        #print "gyroskop_xout: ", ("%5d" % gyroskop_xout), " scaled: ", (gyroskop_xout / 131)
        #print "gyroskop_yout: ", ("%5d" % gyroskop_yout), " scaled: ", (gyroskop_yout / 131)
        #print "gyroskop_zout: ", ("%5d" % gyroskop_zout), " scaled: ", (gyroskop_zout / 131)
        
        acc_xout = self.read_word_2c(0x3b)
        acc_yout = self.read_word_2c(0x3d)
        acc_zout = self.read_word_2c(0x3f)
        
        acc_xout_scaled = acc_xout / 16384.0
        acc_yout_scaled = acc_yout / 16384.0
        acc_zout_scaled = acc_zout / 16384.0
        
        #print "acc_xout: ", ("%6d" % acc_xout), " scaled: ", acc_xout_scaled
        #print "acc_yout: ", ("%6d" % acc_yout), " scaled: ", acc_yout_scaled
        #print "acc_zout: ", ("%6d" % acc_zout), " scaled: ", acc_zout_scaled
        
        xrot=self.get_x_rotation(acc_xout_scaled, acc_yout_scaled, acc_zout_scaled)
        yrot=self.get_y_rotation(acc_xout_scaled, acc_yout_scaled, acc_zout_scaled)
        return (xrot,yrot)

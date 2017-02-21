#!/usr/bin/python
import numpy as np
import usb.core
import time
import signal, sys
import time

from array import array
from pid import PID
from matplotlib import pyplot as plt


# torque constant = 15.8 mNm/A

def sign(x):
    return -1 if x<0 else 1

def parse_angle(angleBytes):
    angle_enc = int(angleBytes[0])+int(angleBytes[1])*256
    #print "Bin: {0:016b} Hex:{1:04x} Dec:{2:0f} Angle:{3:0f}".format(angle_enc, angle_enc, float(angle_enc)/0x3FFF, float(angle_enc)/0x3FFF*360)
    angle_enc = (float(angle_enc) / 0x3FFF * 360) % 360
    return angle_enc

def parse_current(currentBytes):
    resistance = 75e-3
    vmax = 1.65
    gain = 10
    voltage = ((currentBytes[1])*256 + currentBytes[0]) #
    voltage = float(voltage)/0xFFFF - 0.5
    voltage = voltage * 1.65/.5 * vmax/ gain
    return voltage / resistance

class encodertest:

    def __init__(self):
        self.ENC_READ_ANG = 1
        self.TOGGLE_LED = 2
        self.SET_SPEED = 3
        self.SET_CURRENT = 4
        self.READ_CURRENT = 5

        self.dev = usb.core.find(idVendor = 0x6666, idProduct = 0x0003)
        if self.dev is None:
            raise ValueError('no USB device found matching idVendor = 0x6666 and idProduct = 0x0003')
        self.dev.set_configuration()

# AS5048A Register Map
        self.ENC_NOP = 0x0000
        self.ENC_CLEAR_ERROR_FLAG = 0x0001
        self.ENC_PROGRAMMING_CTRL = 0x0003
        self.ENC_OTP_ZERO_POS_HI = 0x0016
        self.ENC_OTP_ZERO_POS_LO = 0x0017
        self.ENC_DIAG_AND_AUTO_GAIN_CTRL = 0x3FFD
        self.ENC_MAGNITUDE = 0x3FFE
        self.ENC_ANGLE_AFTER_ZERO_POS_ADDER = 0x3FFF

    def close(self):
        self.dev = None

    def toggle_led(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED1)
        except usb.core.USBError:
            print "Could not send TOGGLE_LED1 vendor request."
    def read_sw(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_SW1, 0, 0, 1)
        except usb.core.USBError:
            print "Could not send READ_SW1 vendor request."
        else:
            return int(ret[0])
    def enc_readAng(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.ENC_READ_ANG, 0, 0, 2)
        except usb.core.USBError:
            print "Could not send ENC_READ_ANG vendor request."
        else:
            return ret
    def set_duty(self, duty,zero=False):
        # duty from 0.0 ~ 1.0
        try:
            #if zero:
            #    duty = 0
            #else:
            #    duty *= 0.6
            #    duty += -0.4 if duty < 0 else 0.4
            duty = np.uint16(0x3FFF + duty * 0x3FFF)
            ret = self.dev.ctrl_transfer(0xC0, self.SET_SPEED, 0, duty, 0)
        except usb.core.USBError:
            print "Could not send SET_SPEED vendor request."
        else:
            return ret
    def enc_readReg(self, address):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.ENC_READ_REG, address, 0, 2)
        except usb.core.USBError:
            print "Could not send ENC_READ_REG vendor request."
        else:
            return ret
    def read_current(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_CURRENT, 0, 0, 2)
        except usb.core.USBError:
            print "Could not send READ_CURRENT vendor request."
        else:
            return ret

angles = []
times = []

def sigint_handler(signal, frame):
    data = np.c_[angles, times]
    np.savetxt('angles.csv', data, delimiter=',')
    sys.exit(0)

if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler)

    t = encodertest()

    bias = -3
    k = 20./180

    Is = []
    then = time.time()

    while True:
        ang = (parse_angle(t.enc_readAng()) - bias)
        if ang > 180:
            ang -= 360
        elif ang < -180:
            ang += 360

        print ang

        now = time.time()
        dt = now - then
        then = now

        duty = k * ang
        duty = max(min(duty,0.9),-0.9)
        if ang > 0:
            duty = 0
        elif ang < -90:
            duty = -duty
        print 'duty', duty
        t.set_duty(duty)

    t.close()

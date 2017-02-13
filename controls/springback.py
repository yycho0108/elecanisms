#!/usr/bin/python
import numpy as np
import usb.core
import time
import signal, sys
import time

from array import array
from pid import PID
from matplotlib import pyplot as plt

from smoother import Smoother

# torque constant = 15.8 mNm/A

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
    def set_speed(self, speed,zero=False):
        # duty from 0.0 ~ 1.0
        try:
            if zero:
                speed = 0
            #else:
            #    speed *= 0.6
            #    speed += -0.4 if speed < 0 else 0.4
            print speed

            speed = np.uint16(0x3FFF + speed * 0x3FFF)
            ret = self.dev.ctrl_transfer(0xC0, self.SET_SPEED, 0, speed, 0)
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
    pid = PID(0.8,0.05,0.00)
    smoother = Smoother(10) # avg of 100 data
    
    bias = -3

    k = 2./180
    k2 = 0.04
    ang_thresh = 10

    Is = []
    then = time.time()

    ts = []
    dcs = []
    mcs = []
    ss = [] # speed

    plt.ion()

    fig, axs = plt.subplots(2,1)

    g1 = axs[0].plot(ts,dcs)[0] # desired current
    g2 = axs[0].plot(ts,mcs)[0] # measured current plot
    g3 = axs[1].plot(ts, ss, 'r')[0] # speed

    axs[0].autoscale(enable=True,axis='both',tight=False)
    axs[0].legend(['desired current','measured current'])
    axs[1].autoscale(enable=True,axis='both',tight=False)
    axs[1].legend(['speed'])

    while True:
        ang = (parse_angle(t.enc_readAng()) - bias)
        if ang > 180:
            ang -= 360
        elif ang < -180:
            ang += 360

        print 'ang', ang
        I = parse_current(t.read_current())
        Is.append(I)

        if abs(ang) < ang_thresh:
            desired_current = k2 * ang
        else:
            desired_current = k * ang + ang_thresh*(k-k2)*(1 if ang < 0 else -1)

        smoother.put(I)
        measured_current = smoother.get()

        # compute error
        print 'dc : {0:.2f}; mc : {1:.2f}'.format( desired_current, measured_current)
        error = (desired_current - measured_current) #need to counteract

        ## GET DT
        now = time.time()
        dt = now - then
        then = now

        # get speed
        speed = pid.compute(error,dt)
        speed = max(min(speed,1),-1)# capping speed
        ss.append(speed)


        ts.append(now)
        dcs.append(desired_current)
        mcs.append(measured_current)

        g1.set_data(ts,dcs)
        g2.set_data(ts,mcs)
        g3.set_data(ts,ss)

        for ax in axs:
            ax.relim()
            ax.autoscale_view(True,True,True)

        fig.canvas.draw()
        plt.pause(0.010)

        print 'speed : {0:.2f}'.format(speed)

        t.set_speed(speed, zero=(abs(ang)<1))


    t.close()


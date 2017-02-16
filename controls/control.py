#!/usr/bin/python
import numpy as np
import time
import signal, sys
import time

from array import array
from matplotlib import pyplot as plt

from smoother import Smoother
from controller import DamperController, SpringBackController, TextureController, WallController
from pic_interface import PICInterface
from ekf import EKF

# torque constant = 15.8 mNm/A

def parse_angle(angleBytes):
    angle_enc = int(angleBytes[0])+int(angleBytes[1])*256
    #print "Bin: {0:016b} Hex:{1:04x} Dec:{2:0f} Angle:{3:0f}".format(angle_enc, angle_enc, float(angle_enc)/0x3FFF, float(angle_enc)/0x3FFF*360)
    ang = (float(angle_enc) / 0x3FFF * 360) % 360
    if ang > 180:
        ang -= 360
    elif ang < -180:
        ang += 360
    return ang

def parse_current(currentBytes):
    resistance = 75e-3
    vmax = 1.65
    gain = 10
    voltage = ((currentBytes[1])*256 + currentBytes[0]) #
    voltage = float(voltage)/0xFFFF - 0.5
    voltage = voltage * 1.65/.5 * vmax/ gain
    return voltage / resistance

ts = []
angs = []
mcs = []
ds = [] # speed

def sigint_handler(signal, frame):
    data = np.c_[ts, angs, mcs, ds]
    np.savetxt('data.csv', data, delimiter=',')
    sys.exit(0)

def get_time(start):
    return time.time() - start

if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler)

    t = PICInterface()
    smoother = Smoother(50) # avg of 100 data
    ekf = EKF()
    
    bias = -3 # rectify 

    start = time.time()
    then = get_time(start)

    controllers = [TextureController(), SpringBackController(), WallController(), DamperController()]
    controller_names = ['Texture','SpringBack','Wall','Damper']

    TEXTURE,SPRINGBACK,WALL,DAMPER = range(4)
    c_idx = TEXTURE

    last_swapped_behaviors = -10

    while True:

        ############# SENSE BEGIN #############

        ## GET DT w.r.t. elapsed time
        now = get_time(start)
        dt = now - then
        then = now

        # Use Kalman Filter to get better angle/velocity estimates:
        e_ang, e_vel = ekf.predict(ang, dt)[:,0]

        # GET ANGLE Measurements
        ang = (parse_angle(t.enc_readAng()) - bias)
        #print 'ang : {0:.2f}'.format(ang)

        ekf.update(np.atleast_2d(np.deg2rad(ang)))# update observations

        # GET (SMOOTHED) CURRENT
        I = parse_current(t.read_current())
        smoother.put(I)
        measured_current = smoother.get()
        #print 'cur : {0:.2f}'.format(measured_current)

        ############# SENSE END #############

        ############# THINK BEGIN #############
        # Swapping Controllers
        if now - last_swapped_behaviors > 5:
            last_swapped_behaviors = now
            c_idx += 1
            if c_idx >= 4:
                c_idx = 0
            print 'Controller : ', controller_names[c_idx]
            print 'time : {0:.2f} sec.'.format(now)

        # get duty 
        duty = controllers[c_idx].compute(measured_current,e_ang,e_vel,dt)
        duty = max(min(duty,0.9),-0.9)# capping duty 
        #print 'duty: {0:.2f}'.format(duty)
        ############# THINK END #############


        ############# ACT BEGIN #############
        t.set_duty(duty, zero=(abs(ang)<1))
        ############# ACT END #############

        ############# Save Data #############

        ts.append(now)
        angs.append(ang)
        mcs.append(measured_current)
        ds.append(duty)

    t.close()


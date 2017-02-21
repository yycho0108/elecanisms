#!/usr/bin/python
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

from cmath import rect,phase
from math import radians, degrees

def mean_angle(deg):
    return degrees(phase(sum(rect(1, radians(d)) for d in deg)/len(deg))) 

sectorSize = 100

def plotcsv(name):
    data = np.loadtxt(name,delimiter=',')
    df = pd.DataFrame(data, columns=['Time', 'Angle', 'Velocity', 'Measured Current', 'Desired Current'])

    f,ax = plt.subplots(3,sharex=True)
    ax[0].plot(df.Time, df.Angle)
    ax[0].set_title("Position Estimates Over Operation")
    ax[0].set_ylabel("Position (deg)")

    ax[1].plot(df.Time, df.Velocity, color='g')
    ax[1].set_title("Velocity Estimates Over Operation")
    ax[1].set_ylabel("Velocity (rad/s)")

    n = int(np.ceil(max(df.Time)/5))

    for a in ax:
        a.axhline(0,color='black')
        for i in range(n):
            a.axvline(x=5*i,color='gray')

    ax[2].plot(df.Time, df['Measured Current'], label='$I_{m}$')
    ax[2].plot(df.Time, df['Desired Current'], color='g', label='$I_{d}$')
    ax[2].legend(loc=1)
    ax[2].set_title("Current Over Operation")

    ax[2].set_ylabel('Current (A)')
    ax[2].set_xlabel('Time (s)')

    plt.show()

plotcsv('data.csv')

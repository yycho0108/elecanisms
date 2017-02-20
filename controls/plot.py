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
    ax = plt.gca()

    ax.plot(df.Time, df.Angle)
    ax.twinx().plot(df.Time, df.Velocity, color='g')
    plt.axhline(0,color='black')
    plt.title('Vision-Based Encoder Calibration')
    plt.legend(['Angle','Velocity'])
    plt.show()

plotcsv('data.csv')

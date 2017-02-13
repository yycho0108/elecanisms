import numpy as np
import time
from matplotlib import pyplot as plt

class Smoother(object):
    def __init__(self,size):
        self.size = size
        self.l = np.zeros(size)
        self.idx = 0
        self.full = False

    def put(self, v):
        self.l[self.idx] = v
        self.idx += 1
        if self.idx >= self.size:
            self.full = True
            self.idx = 0

    def get(self):
        if self.full:
            return np.mean(self.l)
        else:
            return np.mean(self.l[:self.idx])

if __name__ == "__main__":
    smoother = Smoother(100)
    ts = []
    vs = []
    ss = []

    plt.ion()

    ax = plt.axes()
    ax.autoscale(enable=True,axis='both',tight=False)

    g1 = plt.plot(ts,vs)[0]
    g2 = plt.plot(ts,ss)[0]


    start = time.time()

    while True:
        t = time.time() - start
        v = np.random.normal()
        smoother.put(v)
        s = smoother.get()

        ts.append(t)
        vs.append(v)
        ss.append(s)

        g1.set_data(ts,vs)
        g2.set_data(ts,ss)
        ax.relim()
        ax.autoscale_view(True,True,True)
        plt.draw()
        plt.pause(0.01)

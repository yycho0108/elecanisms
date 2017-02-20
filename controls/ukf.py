from collections import namedtuple
import numpy as np
from scipy.linalg import cholesky, sqrtm
from matplotlib import pyplot as plt

from filterpy.kalman import UnscentedKalmanFilter as UKF2
from filterpy.kalman import MerweScaledSigmaPoints

def sub_ang(a,b, rad=False):
    x = a-b
    d = np.pi if rad else 180.
    return (x+d)%(2*d) -d

def colvec(*args):
    return np.atleast_2d(args).T

def dot(*args):
    return reduce(np.dot, args)


class SigmaPoint(object):
    def __init__(self, n, a=1e-3,b=2,c=0):
        self.n = n
        self.m = 2*n+1
        # a = spread of sigma points, like 1e-3
        # b = 2 is said to be optimal for gaussian
        # c = secondary scaling, "usually set to 0 or 3-n"

        self.a,self.b,self.c = a,b,c
        self.l = a**2 * (n+c) - n 
        #self.sqrt = cholesky
        self.sqrt = sqrtm

    def sigmas(self,x,P):
        n,l = self.n, self.l
        m = self.m
        U = self.sqrt((l+n)*P)

        sigmas = [np.zeros(n) for _ in range(m)]
        sigmas[0] = x
        for k in range(n):
            sigmas[k+1] = x + U[:,[k]] # TODO : fix this for angular!
            sigmas[n+k+1] = x - U[:,[k]]
        return sigmas

    def weights(self):
        n,l = self.n, self.l
        m = self.m

        c = .5 / (n+l)
        w_c = [c for _ in range(m)]
        w_m = [c for _ in range(m)]
        w_c[0] = l / (n+l) + (1-self.a**2 + self.b)
        w_m[0] = l / (n+l)
        return w_m, w_c

    def get(self,x,P):
        return self.sigmas(x,P), self.weights()

#    def get(self,x,P):
#        return self

class UKF(object):
    def __init__(self, n):
        self.x = np.zeros((n,1)) # mean state
        self.S = [] # sigma points
        self.w_m = []
        self.w_c = []
        self.P = np.eye(n) * 1e+1 # covariance
        self.Q = np.eye(n) * 1e-3
        var = (np.pi/180.)**2 * (.0018)
        self.R = np.diag([var]) # based on static angular variance

        self.n = n
        self.sig = SigmaPoint(n)

    def UT(self,S,w_m,w_c,Q):
        n = self.n
        # unscented transform

        #x = np.zeros_like(self.x)
        a = [ (S_i.shape) for S_i in S]
        #print 'a', a
        #w_m = np.array(w_m).reshape(1,2*self.n+1)
        #S = np.array(S)
        #print w_m.shape
        #print S.shape
        #x = np.dot(w_m,S)
        x = np.tensordot(w_m, S,axes=1)
        #x = np.sum([w_i*S_i for w_i,S_i in zip(w_m,S)], axis=0) # "mean" x
        dS = np.array([S_i-x for S_i in S])[:,:,0]

        P = dS.T.dot(np.diag(w_c)).dot(dS) + Q

        #P = np.zeros((n,n))
        #print np.outer(dS,dS)
        #P = np.tensordot(w_c, np.outer(dS,dS), axes=0)
        #print 'a', P[0]
        #P = np.sum([w_i * dS_i.dot(dS_i.T) for w_i,dS_i in zip(w_c,dS)]) + Q 
        #print 'b', P[0]
        return x,P

    def predict(self, dt):
        x, (self.w_m, self.w_c) = self.sig.get(self.x,self.P)
        self.S = [self.f(x_i, dt) for x_i in x] # sigma points
        #print 'x', x[0]
        #print 'S', self.S[0]
        self.x, self.P = self.UT(self.S,self.w_m,self.w_c,self.Q)
        return self.x

    def update(self, z):
        S,w_m,w_c = self.S, self.w_m, self.w_c
        Z = [self.h(S_i) for S_i in S]
        mu_z, P_z = self.UT(Z,w_m,w_c,self.R)
        y = z - mu_z #Innovation - TODO : redefine subtraction?
        P_xz = np.sum([dot(w_i,(S_i-x),(Z_i-mu_z).T) for w_i, S_i, Z_i in zip(w_c,S,Z)], axis=0)
        # P_xz is like P * H.T
        # P_z is like S = H*P*H.T
        K = dot(P_xz,np.linalg.inv(P_z))
        #print 'K', K
        #print 'y', y
        self.x += dot(K,y)
        #print P_z
        self.P -= dot(K, P_z,K.T)
        #print self.P

    def f(self, x, dt):
        #next-state transition
        t,w = x[:,0] #theta, angular velocity
        return colvec(t+w*dt,w)
    
    def h(self,x):
        # measurement mapping : only measures angular position
        return colvec(x[0])


def fx(x,dt):
    t,w = x
    return colvec(t+w*dt,w)[:,0]
def hx(x):
    return np.array(x[0])

if __name__ == "__main__":
    ukf = UKF(2)

    # test
    # s = MerweScaledSigmaPoints(2,alpha=.1,beta=2.,kappa=1.)
    # ukf_2 = UKF2(dim_x=2,dim_z=1,fx=fx,hx=hx,
    #         dt=26/1000., points=s)
    # ukf_2.x = ukf.x[:,0]
    # ukf_2.R = ukf.R.copy()
    # ukf_2.Q = ukf.Q.copy()

    # time
    ts = np.linspace(0,26,1000)

    # estimated state
    e_x = np.random.normal(size=(2,1), scale=1e+1)

    # previous time
    t_p = ts[0]

    xs = np.sin(0.5*ts)
    ws = 0.5*np.cos(0.5*ts) # real
    e_xs = []
    e_ws = []

    var = (np.pi/180.)**2 * (.0018)
    std = np.sqrt(var)

    for x, t in zip(xs,ts):
        # time difference
        dt = (t - t_p)

        # noisy observation
        z = x + np.random.normal(scale=std)

        # prediction
        e_x = ukf.predict(dt)
        #ukf_2.predict(dt)
        #e_x_2 = ukf_2.x

        # update
        ukf.update(z)
        #ukf_2.update(z)

        t_p = t
        e_xs.append(e_x[0])
        e_ws.append(e_x[1])
        #e_ws.append(e_x_2[1])

    ax = plt.gca()
    #ax.plot(xs,e_xs)
    #ax.plot(ts,xs)
    #ax.plot(ts,e_xs)
    #ax2 = ax.twinx()
    ax.plot(ts,ws)
    ax.plot(ts,e_ws)
    plt.axhline(0,color='black')
    plt.show()

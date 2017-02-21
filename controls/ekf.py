import numpy as np
from matplotlib import pyplot as plt

# state definition:
# angular position (float), angular velocity
# sensor: angular position

def colvec(*args):
    return np.atleast_2d(args).T

def dot(*args):
    return reduce(np.dot, args)

class EKF(object):
    def __init__(self):
        # n = size of state vector (= 2)
        # m = size of observation vector (= 1)
        n = 2
        m = 1

        p = 1e+1 # start out with large-ish uncertainty
        q = 1e-3 # assume small process noise

        # Initialization values
        self.x = np.zeros((n,1))

        # (Initial) Covariance
        self.P = np.eye(n) * p

        # Q = Process Noise
        self.Q = np.eye(n) * q

        # R = Measurement Noise - guess 1e-2^2 = 1e-4 (TODO : ascertain)
        var = (np.pi/180.)**2 * (.0018)
        self.R = np.diag([var]) # based on static angular variance

    def predict(self,dt):
        """
        predict(dt) : dt = passage of time
        """

        # Alias names to save typing
        x,P,Q,R = self.x, self.P, self.Q, self.R

        self.x = self.f(x, dt) # dot(B,u), but not used here
        F = self.F(x, dt)
        self.P = dot(F,P,F.T) + Q
        return self.x

    def update(self, z):
        """
        update(z) : z = observations
        """
        # Alias names to save typing
        x,P,Q,R = self.x, self.P, self.Q, self.R
        H = self.H(x)

        y = z - self.h(x) # Y = Measurement "Error" or Innovation
        y = (y + np.pi) % (2*np.pi) - np.pi # remove 0-360 problem

        S = dot(H,P,H.T) + R # S = Innovation Covariance
        K = dot(P,H.T,np.linalg.inv(S)) # K = "Optimal" Kalman Gain
        dx = dot(K,y)
        self.x += dx # Now update x
        self.P -= dot(K,H,P) # Now update P
        return self.x

    def f(self, x, dt):
        #next-state transition
        t,w = x[:,0] #theta, angular velocity
        return colvec(t+w*dt,w)
    
    def F(self,x, dt):
        # Jacobian of f(x), w.r.t. x
        F = np.eye(2)
        F[0,1] = dt # [[1;dt];[0;1]]
        return F

    def h(self,x):
        # measurement mapping : only measures angular position
        return colvec(x[0])

    def H(self,x):
        res = np.zeros((1,2))
        res[0,0] = 1
        return res

def f(t):
    return 0.5 * t - t**2 + np.sin(3*t) + np.sin(t)*np.cos(t)
def f_p(t):
    return 0.5 - 2*t + 3*np.cos(3*t) + (np.cos(t)**2 - np.sin(t)**2)



if __name__ == "__main__":
    ekf = EKF()

    # time
    ts = np.linspace(0,26,1000)

    # estimated state
    e_x = np.random.normal(size=(2,1), scale=1e+1)

    # previous time
    t_p = ts[0]

    xs = f(ts)
    ws = f_p(ts)
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
        e_x = ekf.predict(dt)

        # update
        ekf.update(z)

        t_p = t
        e_xs.append(e_x[0])
        e_ws.append(e_x[1])

    ax = plt.gca()
    #ax.plot(xs,e_xs)
    #ax.plot(ts,xs)
    #ax.plot(ts,e_xs)
    #ax2 = ax.twinx()
    ax.plot(ts,ws)
    ax.plot(ts,e_ws)
    plt.axhline(0,color='black')
    plt.show()

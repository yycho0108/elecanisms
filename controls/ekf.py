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
        self.R = np.diag([1e-4])

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
        S = dot(H,P,H.T) + R # S = Innovation Covariance
        K = dot(P,H.T,np.linalg.inv(S)) # K = "Optimal" Kalman Gain; pinv for numerical stability
        dx = dot(K,y)
        self.x += dx # Now update x
        self.P -= dot(K,H,P) # Now update P
        return self.x

    def f(self, x, dt):
        #next-state transition
        t,w = x[:,0]
        return colvec(t+w*dt,w)
    
    def F(self,x, dt):
        # Jacobian of f(x), w.r.t. x
        F = np.eye(2)
        F[0,1] = dt # [[1;dt];[0;1]]
        return F

    def h(self,x):
        return colvec(x[0])

    def H(self,x):
        res = np.zeros((1,2))
        res[0,0] = 1
        return res

if __name__ == "__main__":
    ekf = EKF()

    # time
    ts = np.linspace(0,13,100)

    # estimated state
    e_x = np.random.normal(size=(2,1), scale=1e+1)
    print e_x

    # previous time
    t_p = ts[0]

    xs = []
    e_xs = []
    e_ws = []

    for t in ts:
        # time difference
        dt = (t - t_p)

        # real state (i.e. real position)
        x = np.sin(t)

        # noisy observation
        z = x + np.random.normal(scale=1e-2)
        
        # prediction
        e_x = ekf.predict(dt)

        # update
        ekf.update(z)

        t_p = t
        xs.append(x)
        e_xs.append(e_x[0])
        e_ws.append(e_x[1])

    ax = plt.gca()
    ax.plot(ts,xs)
    ax.plot(ts,e_xs)
    ax.twinx().plot(ts,e_ws)
    plt.show()

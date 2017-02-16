from abc import ABCMeta, abstractmethod
from pid import PID

class Controller(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass
    @abstractmethod
    def compute(self, cur, ang, vel, dt):
        pass

class SpringBackController(Controller):
    def __init__(self):
        super(SpringBackController,self).__init__()
        self.k = 2./180 # large-angle scaling factor
        self.k2 = 0.05 # small-angle scaling factor
        self.ang_thresh = 10
        self.pid = PID(6.0,0.05,0.00)
    def compute(self, cur, ang, vel, dt):
        k,k2 = self.k, self.k2
        if abs(ang) < self.ang_thresh:
            target_cur = k2 * ang
        else:
            target_cur = k * ang + self.ang_thresh*(k-k2)*(1 if ang < 0 else -1)
        error = (target_cur - cur) #need to counteract
        duty = self.pid.compute(error,dt)
        duty = max(min(duty,0.9),-0.9) 
        return duty

class DamperController(Controller):
    def __init__(self):
        super(DamperController,self).__init__()
        self.k = 1./360
        self.ang = 0.
    def compute(self, cur, ang, vel, dt):
        #vel = (ang - self.ang)/dt
        duty = self.k * vel
        duty = max(min(duty,0.9),-0.9)
        if abs(ang - self.ang) < .15:
            # noise reduction purposes
            duty = 0
        # store previous angle, for velocity computation
        self.ang = ang
        return duty

class TextureController(Controller):
    def __init__(self):
        super(TextureController,self).__init__()
        self.k = 1./360
        self.ang = 0.
    def compute(self, cur, ang, vel, dt):
        #vel = (ang - self.ang)/dt
        duty = self.k * vel
        duty = max(min(duty,0.9),-0.9)
        if (round(ang) % 10) > 5:
            # Texture : change behavior every 5 degrees
            duty = 0
        elif abs(ang - self.ang) < .15:
            # noise reduction purposes
            duty = 0
        # store previous angle, for velocity computation
        self.ang = ang
        return duty

class WallController(Controller):
    def __init__(self):
        super(WallController,self).__init__()
        self.k = 20./180
    def compute(self,cur,ang,vel,dt):
        duty = self.k * ang
        duty = max(min(duty,0.9),-0.9)
        if ang > 0:
            duty = 0
        elif ang < -90:
            duty = -duty
        return duty

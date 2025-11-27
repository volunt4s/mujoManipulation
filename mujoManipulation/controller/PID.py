import numpy as np

class PIDController:
    def __init__(self,
                 K_p,
                 K_i,
                 K_d,
                 dt):
        # Initialize
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.dt = dt
        
        self.err_prev = 0.0
    #TODO
    # I gain
    # anti-windup
    # d term lowpass
    
    def update(self,
               desired,
               curr):
        err = (desired - curr)
        de = (err - self.err_prev) / self.dt
        ut = self.K_p * err + self.K_d * de
        self.err_prev = err
        return ut

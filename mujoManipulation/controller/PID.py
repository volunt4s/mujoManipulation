import numpy as np

class PIDController:
    def __init__(self,
                 K_p,
                 K_i,
                 K_d,
                 dt,
                 tau,
                 robot):
        
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.dt = dt
        self.tau = tau
        self.robot = robot

        self.integral_sum = 0.0

        # For D-gain lowpass
        self.err_prev = 0.0
        self.d_term_prev = 0.0

        # For I-gain anti-windup
        self.ctrl_range = self.robot.model.actuator_ctrlrange

    
    def update(self,
               desired,
               curr):
        err = (desired - curr)
        output_limit_min = self.ctrl_range[:, 0]
        output_limit_max = self.ctrl_range[:, 1]

        # P
        p_term = self.K_p * err

        # I
        i_term_update = self.K_i * err * self.dt

        # D with lowpass
        de_raw = (err - self.err_prev) / self.dt
        alpha = self.tau / (self.tau + self.dt)
        beta = self.dt / (self.tau + self.dt)
        d_term = alpha * self.d_term_prev + beta * (self.K_d * de_raw)

        # calculate control input 
        u_tentative = p_term + self.integral_sum + d_term
        u_final = np.clip(u_tentative, output_limit_min, output_limit_max)

        # apply anti-windup
        is_not_saturated = np.equal(u_tentative, u_final)
        update_mask = i_term_update * is_not_saturated.astype(float)
        self.integral_sum += update_mask
        
        # previous value update
        self.err_prev = err
        self.d_term_prev = d_term

        return u_final
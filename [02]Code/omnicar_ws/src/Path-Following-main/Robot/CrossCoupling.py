import numpy as np
class CrossCoupling:
    def __init__(self, kp, ki, kf, dt, sat):
        self.kp = kp
        self.ki = ki
        self.kd = 0
        self.kf = kf
        self.integral_error = 0
        self.derivative_error = 0
        self.last_error = 0
        self.sat = sat
        self.dt = dt
        self.w_error_integral = 0
        self.A = 64.50
        self.tau = 0.1

    def compute(self, x, ref,x2, x2_ref,flag=1):
        error = ref - x
        error2 = x2_ref - x2
        w_error = error-error2
        self.integral_error += error * self.dt
        self.w_error_integral += w_error *self.dt
        self.derivative_error = (error - self.last_error) / self.dt
        cc_term = 3*self.kp * flag * (w_error) + self.w_error_integral * 0*self.ki * flag
        u = self.kp * error + self.ki * self.integral_error + self.kd * self.derivative_error + self.kf * ref + cc_term

        if u > self.sat:
            self.integral_error -= error * self.dt
            self.w_error_integral -= w_error * self.dt
            u = self.sat
        elif u < -self.sat:
            self.integral_error -= error * self.dt
            #self.w_error_integral -= w_error * self.dt
            u = -self.sat

       # x_k_1 = x + (self.A*u-x)/self.tau*self.dt
        #x_min = -50
        #if x_k_1 <x_min:
         #   u = 1/self.A*(self.tau*(x_min-x)/self.dt+x)
          #  self.w_error_integral -= w_error *self.dt
           # self.integral_error -= error * self.dt



        self.last_error = error
        return u

    def reset(self):
        self.integral_error = 0
        self.derivative_error = 0
        self.last_error = 0
        self.w_error_integral = 0
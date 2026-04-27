class PID:
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

    def compute(self, x, ref):
        error = ref - x
        self.integral_error += error * self.dt
        self.derivative_error = (error - self.last_error) / self.dt
        u = self.kp * error + self.ki * self.integral_error + self.kd * self.derivative_error + self.kf * ref
        if u > self.sat:
            self.integral_error -= error * self.dt
            u = self.sat
        elif u < -self.sat:
            self.integral_error -= error * self.dt
            u = -self.sat
        self.last_error = error
        return u

    def reset(self):
        self.integral_error = 0
        self.derivative_error = 0
        self.last_error = 0
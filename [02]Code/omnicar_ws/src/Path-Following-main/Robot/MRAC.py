class MRAC:
    def __init__(self, A, tau, dt):
        self.Am = -1/(tau)
        self.Bm = 1/(tau)
        self.A = -1 / tau
        self.B = A / tau
        self.kx = (self.Am - self.A) / self.B
        self.kr = self.Bm / self.B
        self.theta = 0
        self.theta2 = 0
        self.xm = 0
        self.gamma_x = 0.00001
        self.gamma_r = 0.000005
        self.gamma_theta = 0.0005
        self.gamma_theta2 = 0.000000001
        self.dt = dt

    def compute(self, x, r):
        u = self.kr * r + self.kx * x + self.theta * 1 + self.theta2 *0**2
        e = x - self.xm
        self.kx = self.kx + (-self.gamma_x * x * e) * self.dt
        self.kr = self.kr + (-self.gamma_r * r * e) * self.dt
        self.theta = self.theta + (-self.gamma_theta * 1 * e)
        self.theta2 = self.theta2 + (-self.gamma_theta2 * 1 * e)


        self.xm = self.xm + (r * self.Bm + self.xm * self.Am) * self.dt
        return u



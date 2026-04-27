import numpy as np
from Robot.MRAC import *
from Robot.PID import *
from Robot.CrossCoupling import *
from dynamic_systems.dynamic_systems import LinearSystem  # or wherever your library is



class Motor(LinearSystem):
    def __init__(self, cfg):
        self.cfg = cfg

        gain = cfg["motor_gain"]
        tau = cfg["motor_time_constant"]

        # Introduce uncertainties
        gain += np.random.normal(0, cfg["gain_mismatch"])
        tau += np.random.normal(0.0, cfg["time_constant_mismatch"])

        # Store constants
        self.gain = gain
        self.tau = tau
        self.bias = np.random.normal(0, cfg["measurement_bias"])
        self.std = cfg["measurement_std"]
        self.max_delta_u = cfg["max_delta_u"]
        self.max_u = cfg["max_u"]
        self.dt_sim = cfg["dt_sim"]
        self.dt_control = cfg["dt_control"]
        self.ref = 0

        # Linear system matrices
        A = np.array([[-1.0 / tau]])
        B = np.array([[gain / tau]])

        # Init LinearSystem
        super().__init__(A=A, B=B, state=np.array([0.0]), control=np.array([0.0]))

        # Controllers
        tau_cl = cfg["closed_loop_constant"]
        kp = (cfg["motor_time_constant"] / tau_cl) / cfg["motor_gain"]
        ki = kp / cfg["motor_time_constant"]
        kf = cfg["kf_percentage"] * 1 / cfg["motor_gain"]

        self.controller2 = PID(kp, ki, kf, self.dt_control, self.max_u)
        self.cc_control = CrossCoupling(kp, ki, kf, self.dt_control, self.max_u)
        self.controller = MRAC(cfg["motor_gain"], cfg["motor_time_constant"], self.dt_control)

        self.w = 0
        self.w_meas = 0

    def update(self):
        ''' Integrate system for one step using dt_sim '''
        self.actuate(self.dt_sim)
        self.w = self.get_state()[0]
        self.w_meas = self.w + np.random.normal(abs(self.w) * self.bias, abs(self.w) * self.std)

    def control(self, w_ref, m=1):
        self.ref = w_ref
        if m == 1:
            U = self.controller.compute(self.w_meas, w_ref)
        else:
            U = self.controller2.compute(self.w_meas, w_ref)

        if U > self.get_control()[0]:
            U = min(self.get_control()[0] + self.max_delta_u, U)
        elif U < self.get_control()[0]:
            U = max(U, self.get_control()[0] - self.max_delta_u)

        U = np.clip(U, -self.max_u, self.max_u)
        self.set_control(np.array([U]))

    def control_cross_coupling(self, w_ref, w2, w2ref, flag):
        self.ref = w_ref
        U = self.cc_control.compute(self.w_meas, w_ref, w2, w2ref, flag)

        if U > self.get_control()[0]:
            U = min(self.get_control()[0] + self.max_delta_u, U)
        elif U < self.get_control()[0]:
            U = max(U, self.get_control()[0] - self.max_delta_u)

        U = np.clip(U, -self.max_u, self.max_u)
        self.set_control(np.array([U]))
import numpy as np
from Robot.Motor import *
import pickle
from controllers.path import *
from dynamic_systems.dynamic_systems import AffineSystem





class DiffDrive(AffineSystem):
    def __init__(self, cfg):
        self.cfg = cfg
        self.dt_sim = cfg["dt_sim"]
        self.dt_control = cfg["dt_control"]
        self.r = cfg["wheel_radius"]
        self.b = cfg["robot_diameter"]
        self.max_v = cfg["v_max"]
        self.w_max = self.max_v / self.r
        self.w_limit = self.w_max + cfg["w_delta_lim"]
        self.t = 0
        self.i = 0
        self.ref = np.array([0.0, 0.0])  # [v, w]
        self.brake = False
        self.epsilon = 0.50

        # Motors
        self.motors = [Motor(cfg), Motor(cfg)]

        # Path follower
        self.PF = Path(cfg)

        # Dynamics functions
        def f(x):
            return np.zeros(3)

        def g(x):
            theta = x[2]
            return np.array([
                [np.cos(theta), -np.sin(theta)*self.epsilon],
                [np.sin(theta), np.cos(theta)*self.epsilon],
                [0, 1]
            ])

        # Initial state [x, y, theta]
        pose = self.PF.get_spline_pose(0)
        state = np.array([pose[0], pose[1], pose[2]])

        # Initial control [v, w]
        control = np.array([0.0, 0.0])

        super().__init__(n=3, m=2, f=f, g=g, state=state, control=control)

    def reset(self):
        pose = self.PF.get_spline_pose(0)
        self.set_state(np.array([pose[0], pose[1], pose[2]]))
        self.set_control(np.array([0.0, 0.0]))
        self.motors = [Motor(self.cfg), Motor(self.cfg)]
        self.t = 0
        self.i = 0

    def step(self,u):
        ''' One simulation/control step '''

        v,w = u
        self.ref = u

        # Convert v, w → w1, w2 (rad/s) for motors
        v1 = v - w * self.b
        v2 = v + w * self.b
        w1_ref = np.clip(v1 / self.r, -self.w_limit, self.w_limit)
        w2_ref = np.clip(v2 / self.r, -self.w_limit, self.w_limit)
        # Apply motor control
        self.motors[0].control(w1_ref, m=0)
        self.motors[1].control(w2_ref, m=0)

        # Simulate motors and robot kinematics
        for _ in range(int(self.dt_control / self.dt_sim)):
            self.motors[0].update()
            self.motors[1].update()

            v1 = self.motors[0].get_state()[0] * self.r
            v2 = self.motors[1].get_state()[0] * self.r

            v_actual = (v1 + v2) / 2
            w_actual = (v2 - v1) / (2 * self.b)

            # Set actual input to affine system
            self.set_control(np.array([v_actual, w_actual]))

            # Simulate robot kinematics
            self.actuate(self.dt_sim)
            self.t += self.dt_sim
            self.i += 1

    def get_pose(self):
        return self.get_state()

    def get_velocities(self):
        return self.get_control()  # [v, w]
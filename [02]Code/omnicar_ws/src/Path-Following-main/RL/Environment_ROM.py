import copy
import random
import time
from typing import Any, Optional

import gymnasium as gym
import numpy as np
from gymnasium.core import ObsType

from controllers.clf_controllers import CLFTrackingQP
from controllers.path import Path
from controllers.cbf_controller import HOCBFQuadraticQP, CBFQuadraticQP
from dynamic_systems.dynamic_systems import LinearSystem
from dynamic_systems.systems import *
from functions.barrier_functions import canonical2D, ConcaveQuadraticBarrier
from optimization_programs.fast_qp import QuadraticProgram

def map_range(x, a1, b1, a2, b2):
    return a2 + (x - a1) * (b2 - a2) / (b1 - a1)

class Environment(gym.Env):
    def __init__(self,cfg):
        self.path = Path(cfg)
        self.dt = cfg["dt_control"]
        self.alpha = 4
        self.eps0 = 24
        self.lambda_issf = -np.log(self.eps0)
        self.lambda_clf = 10
        self.input_filter_K = 10
        self.t = 0
        self.max_v = 10
        self.max_w = 2
        self.max_vp = self.max_v+1
        self.L = 0.5
        self.sys = Unicycle2ndOrder(L=self.L)
        self.ROM = Unicycle(L=self.L)
        self.u = np.array([0,0])
        self.radius = 0.98

        self.u1_bounds = np.array([-self.max_v, self.max_v])
        self.u2_bounds = np.array([-self.max_w, self.max_w])
        ub = np.array([self.u1_bounds[1], self.u2_bounds[1]])
        lb = np.array([self.u1_bounds[0], self.u2_bounds[0]])
        self.A = np.array([
            [1.0, 0.0],  # u1 <= max_v
            [-1.0, 0.0],  # -u1 <= max_v
            [0.0, 1.0],  # u2 <= max_w
            [0.0, -1.0]  # -u2 <= max_w
        ])

        self.d_bounds = np.array([
            self.max_v,
            0,
            self.max_w,
            self.max_w
        ])

        self.P = np.array([[0.1, 0, 0], [0.0, 1.0, 0.0], [0, 0, 0.001]])
        self.Q = np.zeros(3)
        self.QP = QuadraticProgram()
        self.QP.set_cost(P=self.P, q=self.Q)


        H_ = canonical2D([1, 1], np.rad2deg(0.0))
        H = np.zeros((3, 3))
        H[0:2, 0:2] = H_
        p = np.array([0, 0, 0])

        self.cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=0.5,
                                      limits=(-4, 4, -4, 4), spacing=0.05,beta1=self.alpha,ISSF=True,eps0=self.eps0,lambd= self.lambda_issf,dt=self.dt)
        self.cbf = self.path.update_barrier_function(self.cbf,n=3,radius=self.radius)
        self.clf = CLFTrackingQP(3, 2, lambd=self.lambda_clf)
        self.ROM_CBF = CBFQuadraticQP(self.ROM, self.cbf)

        self._obs_agent = np.zeros(2, dtype=np.float32)
        self._obs_R = np.zeros((2, 2), dtype=np.float32)
        self._obs_dict = {
            "agent": self._obs_agent,
            "path": None,
            "barrier": 0.0,
            "t": 0.0,
            "gamma": 0.0,
            "u": np.zeros(2),
            "u_corr": np.zeros(2),
            "vd": 0.0,
            #"t_last_cp": 0.0
        }


        #TO DO ADD INPUT FILTER
        self.observation_space = gym.spaces.Dict(
            {
             "agent": gym.spaces.Box(low=-1,high=1,shape=(2,),dtype=np.float64),
             "path":  gym.spaces.Box(low=-1,high=1, shape =(80,2), dtype=np.float64),
             "barrier": gym.spaces.Box(low=-1,high=1,shape=(1,),dtype=np.float64),
             "t": gym.spaces.Box(low=0, high=1, shape=(1,),dtype=np.float64),
             "gamma": gym.spaces.Box(low=0,high=1, shape=(1,), dtype=np.float64),
             "u_corr": gym.spaces.Box(low=-1, high=5, shape=(2,), dtype=np.float64),
             "u": gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float64),
             "vd": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float64),
             #"t_last_cp": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float64),
            }
        )

        self.action_space = gym.spaces.Box(low=np.array([-1,-1,-1]), high= np.array([1,1,1]), dtype = np.float32  )

        self.terminated = False
        self.truncated = False
        self.observation = None #self._get_obs()
        self.info = None #self._get_info()
        self.action = None
        self.uf = None
        self.u_corr = None
        self.k0 = None
        self.vd = 0
        self.vp = 0
        self.max_t = 500
        self.t_last_cp = 0
        self.next_cp = 0.1
        self.t = 0
        self.vd_nom = 0
        self.u_nom = 0
        self.vd_f = 0
        self.vd_correction = -self.vd_nom
        self.u_nom_correction = np.array([0, 0])
        self.fail = False

    def _get_obs(self):
        state = self.sys.get_state()
        pose_x, pose_y, theta = state[0], state[1], state[2]

        # 1. Update pre-allocated agent array in-place
        self._obs_agent[0] = state[3] / self.max_v
        self._obs_agent[1] = state[4] / self.max_w

        # 2. Update pre-allocated rotation matrix in-place
        c = np.cos(-theta)
        s = np.sin(-theta)
        self._obs_R[0, 0] = c
        self._obs_R[0, 1] = -s
        self._obs_R[1, 0] = s
        self._obs_R[1, 1] = c

        # 3. Vectorized Path Math (Assuming sample_path_ahead returns a NumPy array)
        path = self.path.sample_path_ahead(80, 0.2)

        # Broadcasting subtraction (fast C-level operation)
        path[:, 0] -= pose_x
        path[:, 1] -= pose_y

        # Fast matrix multiplication
        path_robot = path @ self._obs_R.T

        # 4. Update the pre-allocated dictionary
        self._obs_dict["path"] = path_robot.copy()/16
        self._obs_dict["barrier"] = self.cbf(self.ROM.get_state())
        self._obs_dict["t"] = self.t / self.max_t
        self._obs_dict["gamma"] = self.path.gamma /1000
        self._obs_dict["uf"] = self.uf * np.array([1/self.max_v, 1/self.max_w])
        self._obs_dict["u_corr"] = self.u_corr * np.array([1/self.max_v, 1/self.max_w])
        self._obs_dict["vp"] = self.vp / self.max_vp
        #self._obs_dict["t_last_cp"] = self.t_last_cp / self.max_t

        return self._obs_dict

    def _get_info(self):

        cbf={
            "hessian": copy.deepcopy(self.cbf.H),
            "center": copy.deepcopy(self.cbf.center),
            "height": copy.deepcopy(self.cbf.height),
            "limits": (self.cbf.center[0] - 2, self.cbf.center[0] + 2, self.cbf.center[1] - 2, self.cbf.center[1] + 2),
            "spacing": self.cbf.spacing
        }
        info = {
            "state" : self.sys.get_state(),
            "u" : self.u,
            "cbf": cbf,
        }
        return info

    def reset(self ,seed = 111,options=None):
        super().reset(seed=seed)


        if self.path.gamma <0.99*1000 and self.t < self.max_t:
            new_g = max(self.path.gamma-0.05*1000,0)
            self.path.reset(new_g)
            pose = self.path.get_spline_pose(new_g)
            self.sys.set_state([pose[0]-0.01, pose[1]-0.01, pose[2], 0, 0])
            #self.max_t-=  self.t
        else:
            #map = random.choice([ "../splines/Silverstone/Silverstone_centerline_tck.yaml", "../splines/Zandvoort/Zandvoort_centerline_tck.yaml"])
            self.path.reset(10)
            self.next_cp = 0.1
            self.t_last_cp = 0
            self.t = 0
            pose = self.path.get_spline_pose(10)
            self.sys.set_state([pose[0]-0.01, pose[1]-0.01, pose[2], 0, 0])
        self.ROM.set_state(self.sys.get_state()[0:3])
        self.path.get_high_order_terms(gamma_dot=0)
        self.ROM_CBF.cbf = self.path.update_barrier_function(self.cbf, 3, radius=self.radius)

        self.terminated = False
        self.truncated = False
        self.action = None
        self.u_corr = np.array([0,0])
        self.u = np.array([0,0])
        self.uf = np.array([0,0])
        self.k0 = np.array([0,0])
        self.vd = 0
        self.vd_f = 0
        self.vd_corrected = 0
        self.max_t = 500

        self.observation = self._get_obs()
        self.info = self._get_info()
        self.fail = False
        self.vd_f = 0
        return self.observation, self.info

    def step(self, action):

        self.truncated = False
        self.fail = False

        v = map_range(action[0], -1, 1, -self.max_v, self.max_v)
        omega = map_range(action[1], -1, 1, -self.max_w, self.max_w)
        u_nom = np.array([v, omega])
        vp_temp = map_range(action[2], -1, 1, 0, self.max_vp)
        self.vd_nom = self.path.vref_to_gamma_dot(vp_temp)

        for i in range(10):
            x = self.sys.get_state()
            q = x[0:3]
            xi = x[3:5]
            self.path.get_high_order_terms(gamma_dot=self.vd_f)
            self.ROM_CBF.cbf = self.path.update_barrier_function(self.cbf, 3, radius=self.radius)
            self.ROM.set_control(self.uf)
            self.u_corr = self.ROM_CBF.get_smooth_control()
            k0 = self.u_corr
            u_total = k0 + self.uf

            uf_dot = self.input_filter_K * (u_nom - self.uf)
            vd_dot = (self.vd_nom - self.vd_f) * self.input_filter_K * 2
            grad_k0_q = self.ROM_CBF.get_sofptlus_gradient()
            grad_k0_uf = self.ROM_CBF.get_k0_gradient_uf(self.uf)
            grad_k0_gamma = self.ROM_CBF.get_k0_derivative_gamma(self.uf)
            grad_k0_vd = self.ROM_CBF.get_k0_derivative_vd(self.uf)
            x_dot = self.ROM.f(q) + self.ROM.g(q) @ xi

            # FORCE FLATTEN to block NumPy broadcasting traps
            dk0_dq = (grad_k0_q @ x_dot).flatten()
            dk0_dgamma = (grad_k0_gamma * self.vd_f).flatten()

            eta = u_total + (dk0_dq + dk0_dgamma) * self.dt

            C, b = self.build_qp_matrices(eta, uf_dot, vd_dot, grad_k0_uf, grad_k0_vd)
            self.QP.set_inequality_constraints(C, b)
            raw_sol = self.QP.get_solution()

            if raw_sol is None:
                # FALLBACK: If bounds are mathematically impossible, command max braking.
                #u_nom_corrected = np.array([0.0, 0.0])  # Force filters to pull to zero
                vd_corrected = 0.0
                self.vd_correction = -self.vd_nom
                self.u_nom_correction = np.array([0,0])
                u_nom_corrected = u_nom
            else:
                sol = np.array(raw_sol).flatten()
                self.u_nom_correction = (sol[0:2] + uf_dot) / self.input_filter_K + self.uf - u_nom
                self.vd_nom_correction = (sol[2] + vd_dot) / (self.input_filter_K * 2) + self.vd_f - self.vd_nom
                u_nom_corrected = u_nom + self.u_nom_correction
                vd_corrected = self.vd_nom + self.vd_nom_correction


            uf_dot = self.input_filter_K * (u_nom_corrected - self.uf)
            vd_dot = (vd_corrected - self.vd_f) * self.input_filter_K * 2
            dk0_duf = (grad_k0_uf @ uf_dot).flatten()
            dk0_dvddot = (grad_k0_vd * vd_dot).flatten()

            dt_k0 = dk0_dq + dk0_duf + dk0_dgamma + dk0_dvddot
            dt_utotal = dt_k0 + uf_dot.flatten()

            q_dot = self.ROM.g(q) @ xi
            V = self.clf.get_V(q_dot, self.ROM.g(q) @ u_total)
            #print(V)

            u = self.clf.get_control(q_dot, xi, u_total, dt_utotal, self.ROM.g(q), self.ROM.jacobian_g(q), self.sys.g(x)[3:5, :], V)
            # print(ROM.get_state())

            h = self.ROM_CBF.cbf(q) - 1 / 15 * (V - 0.08 / self.alpha * 15)
            # print(ROM_CBF.cbf(q))
            # if h<0 or ROM_CBF.cbf(q) <-0.02059999999999995:
            #   print(h,ROM_CBF.cbf(q), t)

            # 6. Step all systems forward to time t + dt
            self.uf = input_filter(u_nom_corrected, self.uf, self.input_filter_K, self.dt)  # Steps uf forward
            self.sys.step(u, self.dt)  # Steps x forward
            self.path.update_with_gamma(self.vd_f)
            self.vd_f += (vd_corrected - self.vd_f) * self.input_filter_K * self.dt * 2
            self.path.get_high_order_terms(gamma_dot=self.vd_f)
            self.ROM_CBF.cbf = self.path.update_barrier_function(self.ROM_CBF.cbf, 3, radius=self.radius)
            self.ROM.set_state(self.sys.get_state()[0:3])

            self.t += self.dt
            self.vp = self.path.gamma_dot_to_vref(self.vd_f)


        # 7. Check Terminations
        if self.path.gamma > 0.99 * 1000:  # Adjust 1000 based on your track scaling
            self.terminated = True

        if self.t > self.max_t:
            self.truncated = True
            self.fail = True

        state = self.sys.get_state()
        path_pose = self.path.get_spline_pose(self.path.gamma)
        if np.linalg.norm(state[0:2] - path_pose[0:2]) > 1.1:
            self.terminated = True
            self.fail = True
            print(np.linalg.norm(state[0:2] - path_pose[0:2]),V,self.t)


        # 8. Return RL Step
        reward = self.reward_function()
        self.observation = self._get_obs()
        self.info = self._get_info()
        return self.observation, reward, self.terminated, self.truncated, self.info

    def reward_function(self):
        r=0
        alpha =1
        if self.terminated and not self.fail:
            r+=max(0,(6000 - 100*self.t)) #50-self.t/10#r+=1*(500 - self.t) #50-self.t/10
        if self.truncated:
            r+=-(self.max_t )
        if self.fail:
            r+=-50

        # if self.path.gamma > self.next_cp:
        #     r+=max(0,20-(self.t-self.t_last_cp))
        #     self.next_cp+=0.05
        #     self.t_last_cp=self.t

        max_vd = self.path.vref_to_gamma_dot(self.max_vp)
        r+= -alpha*np.linalg.norm(self.u_corr*np.array([1/self.max_v , 1/self.max_w]))
        r += -alpha * np.linalg.norm(self.u_nom_correction * np.array([1 / self.max_v, 1 / self.max_w]))
        r+= -alpha*((self.vd_correction)/max_vd)**2
        r+=  (min(self.max_v-0.5,self.vp)/(self.max_v-0.5))  #np.sign(self.sys.get_state()[3])*(self.sys.get_state()[3]/10)**4/10

        return r

    def build_qp_matrices(self, eta, uf_dot, vd_dot, grad_k0_uf, grad_k0_vd):
        # Dimensions
        dim_u = 2  # Dimension of uf (e.g., [v, w])
        dim_v = 1  # Dimension of vd
        dim_x = dim_u + dim_v

        # 1. Objective Matrices (min ||x||^2)
        P = np.array([[0.1, 0, 0], [0.0, 1.0, 0.0], [0, 0, 0.001]])
        Q = np.zeros(dim_x)

        # Identity matrix for the (I + grad_k0_uf) term
        I_u = np.eye(dim_u)

        # Pre-compute the shared gradient block: (I + grad_k0_uf)
        grad_u_block = I_u + grad_k0_uf

        # Ensure grad_k0_vd is a column vector, shape (2, 1)
        grad_k0_vd_col = grad_k0_vd.reshape(-1, 1)

        # 2. Build C (The constraint matrix)
        # Horizontally stack the multipliers and scale the WHOLE block by dt
        inner_C = np.hstack((grad_u_block, grad_k0_vd_col)) * self.dt
        C = self.A @ inner_C

        # 3. Build b (The bounds vector)
        # Calculate the known drift/nominal terms
        # grad_u_block @ uf_dot yields shape (2,)
        # (grad_k0_vd_col * vd_dot).flatten() yields shape (2,)
        derivative_terms = (grad_u_block @ uf_dot) + (grad_k0_vd_col * vd_dot).flatten()

        known_terms = eta + (derivative_terms * self.dt)

        b = self.d_bounds - self.A @ known_terms

        return  C, b




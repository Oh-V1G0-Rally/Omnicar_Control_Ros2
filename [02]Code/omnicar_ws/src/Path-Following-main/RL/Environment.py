import copy
import random
from typing import Any, Optional

import gymnasium as gym
import numpy as np
from gymnasium.core import ObsType

from controllers.path import Path
from controllers.cbf_controller import HOCBFQuadraticQP
from dynamic_systems.dynamic_systems import LinearSystem
from dynamic_systems.systems import *
from functions.barrier_functions import canonical2D, ConcaveQuadraticBarrier

def map_range(x, a1, b1, a2, b2):
    return a2 + (x - a1) * (b2 - a2) / (b1 - a1)

class Environment(gym.Env):
    def __init__(self,cfg):
        self.path = Path(cfg)
        self.dt = cfg["dt_control"]
        self.t = 0
        self.system = Unicycle2ndOrder()
        self.u = np.array([0,0])


        H_ = canonical2D([1, 1], np.rad2deg(0.0))
        H = np.zeros((5, 5))
        H[0:2, 0:2] = H_
        p = np.array([0, 0, 0, 0, 0])

        self.cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=0.5,
                                      limits=(-4, 4, -4, 4), spacing=0.05,beta1=100,beta2=200,dt=self.dt)
        self.pgamma_upper_norm = self.path.get_p_gamma_upper_bound()
        self.cbf = self.path.update_barrier_function(self.cbf,n=5,radius=1)
        self.controller = HOCBFQuadraticQP(self.system, self.cbf)

        self.observation_space = gym.spaces.Dict(
            {
             "agent": gym.spaces.Box(low=-1,high=1,shape=(2,),dtype=np.float64),
             "path":  gym.spaces.Box(low=-np.inf,high=np.inf, shape =(80,2), dtype=np.float64),
             "barrier": gym.spaces.Box(low=-1,high=1,shape=(1,),dtype=np.float64),
             "t": gym.spaces.Box(low=0, high=1, shape=(1,),dtype=np.float64),
             "gamma": gym.spaces.Box(low=0,high=1, shape=(1,), dtype=np.float64),
             "u_corr": gym.spaces.Box(low=-5, high=5, shape=(2,), dtype=np.float64),
             "u": gym.spaces.Box(low=-5, high=5, shape=(2,), dtype=np.float64),
             "vd": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float64),
             "t_last_cp": gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float64),
            }
        )

        self.action_space = gym.spaces.Box(low=np.array([-1,-1,-1]), high= np.array([1,1,1]), dtype = np.float32  )

        self.terminated = False
        self.truncated = False
        self.observation = None #self._get_obs()
        self.info = None #self._get_info()
        self.action = None
        self.u_corr = None
        self.vd = 0
        self.max_t = 500
        self.t_last_cp = 0
        self.next_cp = 0.1
        self.t = 0
        self.fail = False

    def _get_obs(self):
        state = self.system.get_state()
        v = state[3]
        omega = state[4]
        pose = np.array([state[0], state[1]])
        theta = state[2]
        path = np.array(self.path.sample_path_ahead(80,0.2))
        # Translate to robot-centered coordinates
        path_rel = path - pose
        # Rotation matrix to robot frame (negative theta)
        R = np.array([
            [np.cos(-theta), -np.sin(-theta)],
            [np.sin(-theta), np.cos(-theta)]
        ])
        # Rotate all path points into robot frame
        path_robot = path_rel @ R.T
        barrier = self.cbf(self.system.get_state())
        t = self.t/self.max_t
        gamma = self.path.gamma
        return {
            "agent" : np.array([v/10,omega/2]),
            "path" : path_robot,
            "barrier" : barrier,
            "t" : t,
            "gamma" : gamma,
            "u" : self.u,
            "u_corr" : self.u_corr,
            "vd": self.vd/(10+2),
            "t_last_cp" : self.t_last_cp/self.max_t,
        }

    def _get_info(self):

        cbf={
            "hessian": copy.deepcopy(self.cbf.H),
            "center": copy.deepcopy(self.cbf.center),
            "height": copy.deepcopy(self.cbf.height),
            "limits": (self.cbf.center[0] - 2, self.cbf.center[0] + 2, self.cbf.center[1] - 2, self.cbf.center[1] + 2),
            "spacing": self.cbf.spacing
        }
        info = {
            "state" : self.system.get_state(),
            "u" : self.u,
            "cbf": cbf,
        }
        return info

    def reset(self ,seed = 111,options=None):
        super().reset(seed=seed)


        if self.path.gamma <0.99 and self.t < self.max_t:
            new_g = max(self.path.gamma-0.05,0)
            self.path.reset(new_g)
            pose = self.path.get_spline_pose(new_g)
            self.system.set_state([pose[0]-0.01, pose[1]-0.01, pose[2], 0, 0])
            #self.max_t-=  self.t
        else:
            #map = random.choice([ "../splines/Silverstone/Silverstone_centerline_tck.yaml", "../splines/Zandvoort/Zandvoort_centerline_tck.yaml"])
            self.path.reset()
            self.next_cp = 0.1
            self.t_last_cp = 0
            self.t = 0
            pose = self.path.get_spline_pose(0)
            self.system.set_state([0.01,0.01,pose[2],0,0])

        self.terminated = False
        self.truncated = False

        self.action = None
        self.u_corr = np.array([0,0])
        self.u = np.array([0,0])
        self.vd = 0
        self.max_t = 500

        self.observation = self._get_obs()
        self.info = self._get_info()
        self.fail = False
        return self.observation, self.info

    def step(self,action):
        self.truncated = False
        self.fail= False
        self.action = action
        max_v =10
        v = map_range(action[0],-1,1,-max_v,max_v)
        delta = map_range(action[1],-1,1,-2,2)
        u_nom = np.array([v,delta])
        self.vd = map_range(action[2],-1,1,0.5,max_v+2)
        self.vd_nom = self.vd
        self.system.set_control(u_nom)
        flag = False
        while True:
            self.path.get_high_order_terms(self.vd)
            self.cbf = self.path.update_barrier_function(self.cbf, 5,radius=1)
            self.controller.cbf = self.cbf
            self.u_corr = self.controller.get_control()

            if self.u_corr is None:
                if not flag:
                    self.vd = max_v+2
                    flag= True
                self.vd -=0.5

                if self.vd <0:
                    self.vd = self.vd_nom
                    self.u_corr = np.array([0,0])
                    #self.terminated = True
                    #self.fail = True
                    break
            else :
                break
        self.u = u_nom + self.u_corr
        self.system.step(self.u, self.dt)
        self.path.update_with_vref(self.vd)
        self.path.get_high_order_terms(self.vd)
        self.cbf = self.path.update_barrier_function(self.cbf, 5,radius=1)
        self.controller.cbf = self.cbf

        self.t += self.dt
        if self.path.gamma >0.99:
            self.terminated = True



        if self.t >self.max_t:
            self.truncated= True
            self.fail = True

        state = self.system.get_state()
        path_pose = self.path.get_spline_pose(self.path.gamma)
        print(np.linalg.norm(state[0:2]-path_pose[0:2]))
        if np.linalg.norm(state[0:2]-path_pose[0:2]) >1.1:
            self.terminated = True
            self.fail = True


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

        if self.path.gamma > self.next_cp:
            r+=max(0,20-(self.t-self.t_last_cp))
            self.next_cp+=0.05
            self.t_last_cp=self.t

        r+= -alpha*np.linalg.norm(self.u_corr/10)
        r+= -alpha*((self.vd_nom-self.vd)/12)**2
        r+=  (min(9.5,self.vd)/9.5)/(10)  #np.sign(self.system.get_state()[3])*(self.system.get_state()[3]/10)**4/10

        return r



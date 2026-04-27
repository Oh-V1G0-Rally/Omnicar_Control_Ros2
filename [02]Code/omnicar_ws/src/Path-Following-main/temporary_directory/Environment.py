import gym

from Robot import *
from gym import spaces
from gym.spaces import MultiBinary, Box
import numpy as np
from Robot.DiffDrive import *
class Environment(gym.Env):

    def __init__(self,conf):
        np.random.seed(0)
        self.terminated = False
        self.rob = DiffDrive(conf)
        self.x = np.zeros(5)
        self.y = np.zeros(5)
        self.theta = np.zeros(5)
        self.v = np.zeros(5)
        self.w = np.zeros(5)
        self.wref = np.zeros(5)
        self.cumlative_reward = 0

        # Combine all state variables into a single observation array
        self.state = np.concatenate((self.x, self.y, self.theta, self.v, self.w, self.wref))

        # Define the observation space
        self.observation_space = spaces.Box(
            low=-np.inf,  # or appropriate lower bounds
            high=np.inf,  # or appropriate upper bounds
            shape=self.state.shape,
            dtype=np.float32
        )
        self.action_space = spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)

    def reset(self):
        print(self.rob.x , self.cumlative_reward)
        self.rob.reset()
        self.x = np.zeros(5)
        self.y = np.zeros(5)
        self.theta = np.zeros(5)
        self.v = np.zeros(5)
        self.w = np.zeros(5)
        self.wref = np.zeros(5)

        # Combine all state variables into a single observation array
        self.state = np.concatenate((self.x, self.y, self.theta, self.v, self.w, self.wref),dtype=np.float32)
        self.cumlative_reward = 0
        self.terminated = False
        return self.state

    def reward(self):
        r = -100*self.rob.y**2 - self.rob.theta**2
        if self.rob.x >self.rob.x_limit:
            self.terminated = True
        if abs(self.rob.y) >0.05 or abs(np.degrees(self.rob.theta)) > 15 or self.rob.t>10:
            self.terminated= True
            r -= 10.0
        self.cumlative_reward += r
        return r

    def step(self,action):

        self.rob.step(action)
        self.update_state(self.rob.x,self.rob.y,self.rob.theta,self.rob.v_meas,self.rob.w_meas,self.rob.wref)

        return self.state,self.reward(), self.terminated, {}


    def update_state(self, new_x, new_y, new_theta, new_v, new_w, new_wref):
        # Update each state variable with the new values, shifting previous values down
        self.x = np.roll(self.x, shift=1)
        self.y = np.roll(self.y, shift=1)
        self.theta = np.roll(self.theta, shift=1)
        self.v = np.roll(self.v, shift=1)
        self.w = np.roll(self.w, shift=1)
        self.wref = np.roll(self.wref, shift=1)

        # Insert new values at the first index
        self.x[0] = new_x
        self.y[0] = new_y
        self.theta[0] = new_theta
        self.v[0] = new_v
        self.w[0] = new_w
        self.wref[0] = new_wref
        self.state = np.concatenate((self.x, self.y, self.theta, self.v, self.w, self.wref),dtype=np.float32)


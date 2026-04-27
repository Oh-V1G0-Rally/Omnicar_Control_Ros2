import scipy
import inspect

import numpy as np
import scipy.linalg
from scipy.integrate import ode
from typing import Callable


class ControlSystem():
    '''
    General class  for control system.
    Has all the functionality for simulating dynamic systems using scipy integration methods.
    Child classes for specific control system dynamics can be easily build from this one.
    '''

    def __init__(self, **kwargs):

        self.mODE = ode(self.get_flow).set_integrator('dopri5')

        # Specify initial state
        if "n" in kwargs.keys():
            x0 = np.zeros(kwargs["n"])
        if "state" in kwargs.keys():
            x0 = kwargs["state"]

        self.n = len(x0)
        self.set_state(x0)

        # Specify initial control
        if "m" in kwargs.keys():
            u0 = np.zeros(kwargs["m"])
        if "control" in kwargs.keys():
            u0 = kwargs["control"]

        self.m = len(u0)
        self.set_control(u0)

        # Specify system dynamics
        if "dynamics" in kwargs.keys():
            self.dynamics = kwargs["dynamics"]
            iscallable = isinstance(self.dynamics, Callable)
            signature = inspect.signature(self.dynamics)
            hassignature = "x" in signature.parameters.keys() and "u" in signature.parameters.keys()
            if (not iscallable) or (not hassignature):
                raise TypeError("Passed dynamics must be a callable method with 'x' and 'u' input variables.")

        if not hasattr(self, "n"):
            raise Exception("State dimension was not specified!")

        if not hasattr(self, "m"):
            raise Exception("Control dimension was not specified!")

        if not hasattr(self, "dynamics"):
            raise Exception("Control dynamics was not specified!")

        # Initialize state logs
        self.state_log = []
        for _ in range(0, self.n):
            self.state_log.append([])

        # Initialize control logs
        self.control_log = []
        for _ in range(0, self.m):
            self.control_log.append([])

    def set_state(self, state):
        ''' Sets system state '''

        if isinstance(state, list):
            state = np.array(state)

        self._state = state
        self.mODE.set_initial_value(self._state)

    def set_control(self, control_input):
        '''Sets system control '''

        if isinstance(control_input, list):
            control_input = np.array(control_input)
        self._control = control_input

    def actuate(self, dt):
        ''' Integrates the state vector '''
        self._state = self.mODE.integrate(self.mODE.t + dt)
        #self.log_state()
        #self.log_control()

    def log_state(self):
        ''' Logs the state '''
        for state_dim in range(0, self.n):
            self.state_log[state_dim].append(self._state[state_dim])

    def log_control(self):
        ''' Logs the control '''
        for ctrl_dim in range(0, self.m):
            self.control_log[ctrl_dim].append(self._control[ctrl_dim])

    def get_flow(self, t):
        ''' Returns current system flow, that is, its state derivative '''
        return self.dynamics(x=self._state, u=self._control)

    def get_state(self):
        ''' Returns the current system state '''
        return self._state

    def get_control(self):
        ''' Gets the last control input '''
        return self._control
    def step(self,u,dt):
        self.set_control(u)
        self.actuate(dt)
        self.set_control(np.zeros_like(u))


class AffineSystem(ControlSystem):
    '''
    General class for an affine control system dx = f(x) + g(x) u.
    '''

    def __init__(self, f=Callable, g=Callable, jacobian_f=None, jacobian_g= None, **kwargs):
        if not isinstance(f, Callable) or not isinstance(g, Callable):
            raise TypeError("f(x) and g(x) must be callable methods.")

        self.jacobian_fun = jacobian_f
        self.jacobian_fun_g = jacobian_g
        self.f_fun = f
        self.g_fun = g

        affine_dynamics = lambda x, u: self.f(x) + self.g(x) @ u
        super().__init__(dynamics=affine_dynamics, **kwargs)

    def f(self, state):
        ''' Gets f(x) value for the given state '''
        return self.f_fun(state)

    def g(self, state):
        ''' Gets g(x) value for the given state '''
        return self.g_fun(state)

    def jacobian_f(self,state):
        if self.jacobian_fun is not None:
            return self.jacobian_fun(state)
        else:
            raise TypeError("Jacobian is not defined")

    def jacobian_g(self, state):
        if self.jacobian_fun_g is not None:
            return self.jacobian_fun_g(state)
        else:
            raise TypeError("Jacobian g is not defined")



class LinearSystem(AffineSystem):
    '''
    Implements a linear system dx = A x + B u
    '''

    def __init__(self, A, B, **kwargs):

        if not isinstance(A, np.ndarray) or A.ndim != 2:
            raise TypeError("A must be a n x n square matrix.")

        if not isinstance(B, np.ndarray) or B.ndim != 2:
            raise TypeError("B must be a n x m square matrix.")

        if A.shape[0] != A.shape[1]:
            raise TypeError("A matrix must be square.")
        n = A.shape[0]

        if n != B.shape[0]:
            raise TypeError("B matrix must have the same number os rows as the state dimension.")
        m = B.shape[1]

        f = lambda x: A @ x
        g = lambda x: B

        super().__init__(n=n, m=m, f=f, g=g, **kwargs)
        self.A = A
        self.B = B

    def update(self, **kwargs):

        if "A" in kwargs.keys():
            self.A = kwargs["A"]
            self.f_fun = lambda x: self.A @ x
            self.dynamics = lambda x, u: self.A @ x + self.B @ u

        if "B" in kwargs.keys():
            self.B = kwargs["B"]
            self.dynamics = lambda x, u: self.A @ x + self.B @ u



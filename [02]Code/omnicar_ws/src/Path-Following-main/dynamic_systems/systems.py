from dynamic_systems import *
from dynamic_systems.dynamic_systems import AffineSystem
import numpy as np

class Unicycle(AffineSystem):
    def __init__(self, state = np.array([-0.01,0.0,0]), L =0.1):

        self.L = L
        control = np.array([0,0])
        def f(x):
            return np.zeros(3)

        def g(x):
            theta = x[2]
            return np.array([
                [np.cos(theta), -np.sin(theta)*self.L],
                [np.sin(theta), np.cos(theta)*self.L],
                [0, 1]
            ])

        def jacobian_f(x):
            return np.zeros((3,3))

        def jacobian_tensor_g(x):
            """
            Returns the full 3D Jacobian tensor of g(x) with respect to the state x.
            Output shape: (3, 2, 3) -> (rows of g, columns of g, size of state x)
            """
            def dg_dtheta(x):
                """
                Returns the partial derivative of g(x) with respect to theta (x[2]).
                Output shape: (3, 2)
                """
                theta = x[2]
                return np.array([
                    [-np.sin(theta), -np.cos(theta) * self.L],
                    [np.cos(theta), -np.sin(theta) * self.L],
                    [0.0, 0.0]
                ])
            # Initialize the tensor with zeros since dg/dx[0] and dg/dx[1] are 0
            J = np.zeros((3, 2, 3))

            # Populate the slice corresponding to the derivative with respect to x[2]
            J[:, :, 2] = dg_dtheta(x)

            return J



        super().__init__(n=3, m=2, f=f, g=g, state=state, control=control,jacobian_f=jacobian_f,jacobian_g=jacobian_tensor_g)


class Unicycle2ndOrderMRAC(AffineSystem):
    def __init__(self, state = np.array([0.01,0.01,0,0,0]), L=0.5, tau=0.2 ):

        self.L= L
        control = np.array([0,0])
        def f(x):
            theta = x[2]
            v = x[3]
            w = x[4]
            return np.array(
                [v * np.cos(theta) - w*np.sin(theta) * self.L, v * np.sin(theta) + w * np.cos(theta) * self.L, w, -v / tau, -w / tau])

        def g(x):
            return np.array([
                [0, 0],
                [0, 0],
                [0, 0],
                [1 / tau, 0],
                [0, 1 / tau],

            ])

        def jacobian_f(x):
            theta = x[2]
            v = x[3]
            w = x[4]

            return np.array([
                [0, 0, - v * np.sin(theta) - w * self.L* np.cos(theta), np.cos(theta) , - self.L* np.sin(theta)],
                [0, 0, v * np.cos(theta) - w * self.L* np.sin(theta), np.sin(theta),  self.L* np.cos(theta)],
                [0, 0, 0, 0, 1],
                [0, 0, 0, -1/tau, 0],
                [0, 0, 0, 0, -1/tau],
            ])



        super().__init__(n=5, m=2, f=f, g=g, state=state, control=control,jacobian_f = jacobian_f)

class Unicycle2ndOrder(AffineSystem):
    def __init__(self, state = np.array([0.01,0.01,0,0,0]), L=0.5):

        self.L= L
        control = np.array([0,0])
        def f(x):
            theta = x[2]
            v = x[3]
            w = x[4]
            return np.array(
                [v * np.cos(theta) - w*np.sin(theta) * self.L, v * np.sin(theta) + w * np.cos(theta) * self.L, w, 0, 0])

        def g(x):
            return np.array([
                [0, 0],
                [0, 0],
                [0, 0],
                [1 , 0],
                [0, 1],

            ])

        def jacobian_f(x):
            theta = x[2]
            v = x[3]
            w = x[4]

            return np.array([
                [0, 0, - v * np.sin(theta) - w * self.L* np.cos(theta), np.cos(theta) , - self.L* np.sin(theta)],
                [0, 0, v * np.cos(theta) - w * self.L* np.sin(theta), np.sin(theta),  self.L* np.cos(theta)],
                [0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
            ])

        def jacobian_g(state):
            return np.array([
                [0, 0],
                [0, 0],
                [0, 0],
                [1, 0],
                [0, 1],
            ])

        super().__init__(n=5, m=2, f=f, g=g, state=state, control=control,jacobian_f = jacobian_f)

class Integrator(AffineSystem):
    def __init__(self,n=2, state= None, control = None):
        if state is None:
            state = np.ones(n)*0.01
        if control is None:
            control = np.zeros(n)
        def f(x):
            return np.zeros(n)
        def g(x):
            return np.eye(n)
        def jacobian_f(x):
            return np.eye(n)*0
        super().__init__(n=n,m=n, f=f,g=g,state=state,control=control,jacobian_f=jacobian_f)

class DoubleIntegrator(AffineSystem):
    def __init__(self, state= np.array([0.01,0.01,0,0]), control = np.array([0,0])):
        def f(x):
            vx = x[2]
            vy = x[3]
            return np.array([vx,vy,0,0] )
        def g(x):
            return np.array([
                [0, 0],
                [0, 0],
                [1, 0],
                [0, 1]
            ])
        def jacobian_f(x):
            return np.array([
                [0,0,1,0],
                [0,0,0,1],
                [0,0,0,0],
                [0,0,0,0]
            ])
        super().__init__(n=4,m=2, f=f,g=g,state=state,control=control,jacobian_f=jacobian_f)


class Bicycle(AffineSystem):
    def __init__(self, state = np.array([0.01,0.01,0,0]), L = 0.5 ):

        self.L = L
        control = np.array([0,0])
        def f(x):
            v = x[2]
            theta = x[3]
            return np.array(
                [v * np.cos(theta) , v * np.sin(theta) , 0, 0])

        def g(x):
            v = x[2]
            return np.array([
                [0, 0],
                [0, 0],
                [1, 0],
                [0, v/L],


            ])

        def jacobian_f(x):
            theta = x[3]
            v = x[2]


            return np.array([
                [0, 0, np.cos(theta)  , -v* np.sin(theta)],
                [0, 0,np.sin(theta) , v*np.cos(theta)],
                [0, 0, 0, 0 ],
                [0, 0, 0, 0],

            ])

        super().__init__(n=4, m=2, f=f, g=g, state=state, control=control,jacobian_f = jacobian_f)

class BicycleInnerLoop(AffineSystem):
    def __init__(self, state=np.array([0.01, 0.01, 0, 0]), L=0.5, tau=0.2):
        self.L = L
        control = np.array([0, 0])

        def f(x):
            v = x[2]
            theta = x[3]
            return np.array(
                [v * np.cos(theta), v * np.sin(theta), -v / tau, 0])

        def g(x):
            v = x[2]
            return np.array([
                [0, 0],
                [0, 0],
                [1 / tau, 0],
                [0, v / L],

            ])

        def jacobian_f(x):
            theta = x[3]
            v = x[2]

            return np.array([
                [0, 0, np.cos(theta), -v * np.sin(theta)],
                [0, 0, np.sin(theta), v * np.cos(theta)],
                [0, 0, -1/tau, 0],
                [0, 0, 0, 0],

            ])

        super().__init__(n=4, m=2, f=f, g=g, state=state, control=control, jacobian_f=jacobian_f)

# class InputFilter(AffineSystem):
#     def __init__(self,n=2, state= None, control = None, K=1):
#         self.K = K
#         if state is None:
#             state = np.ones(n)*0
#         if control is None:
#             control = np.zeros(n)
#         def f(x):
#             return -x*K
#         def g(x):
#             return np.eye(n)*K
#         def jacobian_f(x):
#             return K
#         super().__init__(n=n,m=n, f=f,g=g,state=state,control=control,jacobian_f=jacobian_f)

def input_filter(u,uf,K=50,dt=1/50):
    return uf + K*(u-uf)*dt
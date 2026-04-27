from platform import system

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import yaml

from Robot.DiffDrive import DiffDrive
#from controllers.cbf_controller import *
#from controllers.clf_controllers import CLFQuadraticQP, CLFCBFQuadraticQP
from dynamic_systems.dynamic_systems import AffineSystem, LinearSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
import copy
import numpy as np

from optimization_programs.equilibria_finder import EquilibriaFinder

system = Integrator(n=2, state=np.array([0.02436485, -0.79888914]))

# H = canonical2D([1,1], np.rad2deg(0))
# xd = np.array([-0.0, 0])
# clf = QuadraticLyapunov(hessian=H, center=xd, height=0,
#                         limits=(-4, 4, -4, 4), spacing=0.05, alpha=0.5)
#
# H = canonical2D([3, 20], np.rad2deg(0.3))
# p = np.array([0, -0.5])
# cbf = QuadraticBarrier(hessian=H, center=p, height=1,
#                         limits=(-4, 4, -4, 4), spacing=0.05,beta1=1)
#
#
#
# solver = EquilibriaFinder(clf=clf,cbf=cbf,plant=system)
# roots = solver.solve(plot= True)
# print(roots)
# roots = solver.filter_non_repulsive_equilibria(roots)
#
# np.dot(clf.gradient(roots[0]), clf.gradient(roots[1]))


# 1. Parameters from Paper (Page 12, Example 4.6)
xc_scalar = 2.0
r = 1.0
p1 = 1.0
p2 = 6.0
p3 = 1.0

# 2. System Matrices (Eq 1502 / 1471)
# A matrix
A = np.array([
    [-p1, 0.0, 0.0],
    [0.0, -p2, p3],
    [0.0, -p3, -p2]
])
A_clf = np.array([
    [-p1, 0.0, 0.0],
    [0.0, -p2, p3],
    [0.0, -p3, -p2]
])*(-2)

# B matrix (Invertible, assumed Identity for G(x)=B^T B to work simply)
B = np.eye(3)

system = LinearSystem(A,B)
H = np.eye(3)
p = np.array([xc_scalar,0,0])
cbf = QuadraticBarrier(hessian=H, center=p, height=1,
                       limits=(-1, 1, -1, 1), spacing=0.05, beta1=1)
clf = QuadraticLyapunov(hessian=A_clf, center=p*0, height=0,
                       limits=(-1, 1, -1, 1), spacing=0.05, alpha=1)



solver = EquilibriaFinder(clf=clf,cbf=cbf,plant=system,dim=3)
roots = solver.solve(plot=  True)
#roots= solver.filter_non_repulsive_equilibria(roots)
print(roots)

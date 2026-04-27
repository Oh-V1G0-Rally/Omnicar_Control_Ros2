from platform import system

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import yaml

from Robot.DiffDrive import DiffDrive
from controllers.cbf_controller import *
from dynamic_systems.dynamic_systems import AffineSystem
from dynamic_systems.systems import *
from functions.barrier_functions import *
from controllers.path import *
import copy


system = Integrator(n=3)
system = Unicycle()

with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())
#system = DiffDrive(conf)
path = Path(conf)
dt = conf["dt_control"]
H = canonical2D([0.5, 3], np.rad2deg(0.5))
p = np.array([0, 0])
cbf = ConcaveQuadraticBarrier(hessian=H, center=p, height=1,
                        limits=(-4, 4, -4, 4), spacing=0.05)
cbf = path.update_barrier_function(cbf)

cbf_log = []
controller = PathCBFQuadraticQP(system, cbf, beta=0.001, dt=dt)

# === Data storage ===
x_log = []
y_log = []
n = 0
# === Simulation ===
path.get_high_order_terms(2)
cbf = path.update_barrier_function(cbf,3)
controller.cbf = cbf
s0_log = []
u0_log = []
system.set_state(np.array([-0.01,0.1,0]))
for _ in range(int(1*10*1/dt)+3):
    s = system.get_state()
    u = controller.get_control()
    path.get_high_order_terms(2)
    cbf = path.update_barrier_function(cbf, 3)
    controller.cbf = cbf
    # log state[0] and control[0]
    s0_log.append(s[0])
    u0_log.append(u[0])

    # existing logging
    x_log.append(s[0])
    y_log.append(s[1])
    system.set_state(np.array([s[0]+0.001*dt,s[1],0]))
    cbf_log.append({
        "hessian": copy.deepcopy(cbf.H),
        "center": copy.deepcopy(cbf.center),
        "height": copy.deepcopy(cbf.height),
        "limits": (cbf.center[0] - 2, cbf.center[0] + 2, cbf.center[1] - 2, cbf.center[1] + 2),
        "spacing": cbf.spacing
    })
    print(cbf(s),u)
    if cbf(s)<0:
        n+=1

# Plot results
t = [i*dt for i in range(len(s0_log))]

plt.figure(figsize=(10,5))

plt.plot(s0_log,u0_log, label="s[0]")
plt.ylabel("Control effort s[0]")
plt.xlabel("state s[0]")
plt.grid(True)
plt.legend()



plt.show()

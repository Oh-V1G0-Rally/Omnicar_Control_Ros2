import random
import  numpy as np
from Robot import *
import yaml
import matplotlib.pyplot as plt
import numpy as np
from Robot.DiffDrive import *
with open("../config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file


np.random.seed(1111)
rob = DiffDrive(conf)
times = []
angular_velocities1 = []
angular_velocities2 = []
error = []
u1 = []
u2 = []
t = 0
ref = 0
for k in range(int(1 * 1 / rob.dt_control)):
    ref=100
    rob.motors[0].control(ref)
    rob.motors[1].control(ref)
    for i in range(int(rob.dt_control / rob.dt_sim)):
        rob.motors[0].update()
        rob.motors[1].update()
        times.append(t)
        angular_velocities1.append(rob.motors[0].w)
        angular_velocities2.append(rob.motors[1].w)
        u1.append(rob.motors[0].get_control())
        u2.append(rob.motors[1].get_control())
        error.append(rob.motors[0].w- rob.motors[1].w)

        t += rob.dt_sim

    # Plotting the results
plt.plot(times, angular_velocities1, label='motor1')
plt.plot(times, angular_velocities2, label='motor2')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (w)')
plt.title('Motor Angular Velocity Over Time')
plt.grid(True)
plt.legend()  # This will display the legend
plt.show()

plt.plot(times, u1, label='Slower Motor')
plt.plot(times, u2, label='Faster Motor')
plt.plot(times, error, label='error')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (w)')
plt.title('Motor Angular Velocity Over Time')
plt.grid(True)
plt.legend()  # This will display the legend
plt.show()

import yaml

from plotting.plot_utils import plot_controls, animate_cbf_robot

with open("../config/config_RL.yaml", "rb") as f:
    conf = yaml.safe_load(f.read())


from RL.Environment_ROM import *


env = Environment(conf)
x_log, y_log, cbf_log, obs_log, theta_log = [], [], [], [], []
v_log, delta_log, vd_log = [], [], []
v_corr_log, w_corr_log = [], []
t_log = []
obs = env.reset()
action = np.array([0,0,0.8])
while True:
    obs, reward, terminated, truncated, info = env.step(action)
    #print(env.sys.get_state())

    if terminated or truncated:
        break


    x_log.append(info["state"][0])
    y_log.append(info["state"][1])
    cbf_log.append(info["cbf"])
    obs_log.append(env.observation["path"]*16)
    path = env.observation["path"]
    theta_log.append(info["state"][2])
    v_log.append(info["state"][3])
    delta_log.append(info["state"][4])
    vd_log.append(env.vp)
    t_log.append(env.t)
    v_corr_log.append(env.u_corr[0])
    w_corr_log.append(env.u_corr[1])


    # Animate robot + centerline
print(env.path.gamma,env.t)

animate_cbf_robot(x_log, y_log, theta_log, cbf_log, v_log, delta_log, vd_log,obs_log=obs_log,dt=10) # overlay centerline)
#plot_trajectories(x_log,y_log,v_log,track["csv"])
plot_controls(v_log,delta_log,vd_log,t_log)
plt.close("all")
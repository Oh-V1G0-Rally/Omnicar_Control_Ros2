from scipy import interpolate
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import yaml

# === Load CSV ===

Track = "Silverstone"
csv_path = Track + "/" + Track + "_centerline.csv"  # path to your CSV
df = pd.read_csv(csv_path, comment='#', header=None)
df.columns = ['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m']

# === Extract centerline coordinates ===
x = df['x_m'].values
y = df['y_m'].values
w_r = df['w_tr_right_m'].values
w_l = df['w_tr_left_m'].values

# === Compute tangents and normals ===
# Forward difference (approximate tangent)
dx = np.gradient(x)
dy = np.gradient(y)
norms = np.sqrt(dx**2 + dy**2)
dx /= norms
dy /= norms

# Normal vectors (perpendicular to tangent)
nx = -dy
ny = dx

# === Compute boundary lines ===
left_x = x + nx * w_l
left_y = y + ny * w_l
right_x = x - nx * w_r
right_y = y - ny * w_r

########################################################################################################################
####################### Interpolate Centerline
l=len(x)
k= 3
t=np.linspace(0,1,l-k+1,endpoint=True)
t=np.append(np.zeros(k),t)
t=np.append(t,np.ones(k))
tck=[t,[x,y],k]
gammas = np.linspace(0, 1, 5000, endpoint=True) #50000

tck_data = {
    't': t.tolist(),
    'x': x.tolist(),
    'y': y.tolist(),
    'k': int(k),
}
with open(Track + "/" + Track+ "_centerline_tck.yaml", 'w') as f:
    yaml.dump(tck_data, f)

########################################################################################################################
####################### Interpolate Right Boundary

tck_data = {
    't': t.tolist(),
    'x': right_x.tolist(),
    'y': right_y.tolist(),
    'k': int(k),
}
with open(Track + "/" + Track+ "_right_boundary_tck.yaml", 'w') as f:
    yaml.dump(tck_data, f)


########################################################################################################################
####################### Interpolate Left Boundary

tck_data = {
    't': t.tolist(),
    'x': left_x.tolist(),
    'y': left_y.tolist(),
    'k': int(k),
}
with open(Track + "/" + Track+ "_left_boundary_tck.yaml", 'w') as f:
    yaml.dump(tck_data, f)



# plt.plot(path[0], path[1], 'b-', lw=1, alpha=0.4, label='BSpline')
# #plt.plot(x, y, 'k-', label='Centerline')
# plt.plot(right_x,right_y)
# plt.plot(left_x,left_y)
#
# plt.gca().set_aspect('equal')
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.title("Track Centerline and Boundaries")
# plt.legend()
# plt.grid(True)
# plt.show()
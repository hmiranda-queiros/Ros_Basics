import math
import pandas as pd
import matplotlib.pyplot as plt

kx = 0.35 / 0.22
ky = 0.2 / 0.13

headers_path = ['x', 'y', 'theta']

df_path_real = pd.read_csv('real_path.csv', names=headers_path)
x_r = df_path_real['x'] * kx
y_r = df_path_real['y'] * ky
theta_r = df_path_real['theta'] * 180/math.pi

df_path_simu = pd.read_csv('simu_path.csv', names=headers_path)
x_s = df_path_simu['x']
y_s = df_path_simu['y']
theta_s = df_path_simu['theta'] * 180/math.pi

headers_way = ['x', 'y']
df_way = pd.read_csv('path_waypoint.csv', names=headers_way)
x_w = df_way['x'] * kx
y_w = df_way['y'] * ky

# plot for real path
f, axs = plt.subplots(1)
axs.plot(x_r, y_r, 'c--', label="real path")
axs.plot(x_r[0], y_r[0], 'go')
axs.annotate("Start", (x_r[0], y_r[0]), fontsize=13)

for i in range(len(x_w)):
    x_i = x_w[i]
    y_i = y_w[i]
    if i == 0:
        axs.plot(x_i, y_i, 'ro', label="way points")
        axs.annotate(i, (x_i, y_i), fontsize=14)
    else:
        axs.plot(x_i, y_i, 'ro')
        axs.annotate(i, (x_i, y_i), fontsize=14)

axs.plot([0.0165, 0.0165, -0.0165, -0.0165, 0.0165], [-0.0325, 0.0325, 0.0325, -0.0325, -0.0325], "b", label="obstacle")

axs.set_aspect('equal')
axs.set_xlim([-0.40, 0.40])
axs.set_ylim([-0.30, 0.30])
axs.set_xlabel("x")
axs.set_ylabel("y")
axs.set_title("Real path")
axs.legend(loc='upper left')
f.show()
f.savefig('real_path.png')

# plot for simulated path
f, axs = plt.subplots(1)
axs.plot(x_s, y_s, 'm--', label="simulated path")
axs.plot(x_s[0], y_s[0], 'go')
axs.annotate("Start", (x_s[0], y_s[0]), fontsize=13)

for i in range(len(x_w)):
    x_i = x_w[i]
    y_i = y_w[i]
    if i == 0:
        axs.plot(x_i, y_i, 'ro', label="way points")
        axs.annotate(i, (x_i, y_i), fontsize=14)
    else:
        axs.plot(x_i, y_i, 'ro')
        axs.annotate(i, (x_i, y_i), fontsize=14)

axs.plot([0.0165, 0.0165, -0.0165, -0.0165, 0.0165], [-0.0325, 0.0325, 0.0325, -0.0325, -0.0325], "b", label="obstacle")

axs.set_aspect('equal')
axs.set_xlim([-0.40, 0.40])
axs.set_ylim([-0.30, 0.30])
axs.set_xlabel("x")
axs.set_ylabel("y")
axs.set_title("Simulated path")
axs.legend(loc='upper left')
f.show()
f.savefig('simulated_path.png')


# plot for real path vs simulated path
f, axs = plt.subplots(1)
axs.plot(x_r, y_r, 'c--', label="real path")
axs.plot(x_r[0], y_r[0], 'go')
axs.annotate("Start", (x_r[0], y_r[0]), fontsize=13)

axs.plot(x_s, y_s, 'm--', label="simulated path")

for i in range(len(x_w)):
    x_i = x_w[i]
    y_i = y_w[i]
    if i == 0:
        axs.plot(x_i, y_i, 'ro', label="way points")
        axs.annotate(i, (x_i, y_i), fontsize=14)
    else:
        axs.plot(x_i, y_i, 'ro')
        axs.annotate(i, (x_i, y_i), fontsize=14)

axs.plot([0.0165, 0.0165, -0.0165, -0.0165, 0.0165], [-0.0325, 0.0325, 0.0325, -0.0325, -0.0325], "b", label="obstacle")

axs.set_aspect('equal')
axs.set_xlim([-0.40, 0.40])
axs.set_ylim([-0.30, 0.30])
axs.set_xlabel("x")
axs.set_ylabel("y")
axs.set_title("Real path vs Simulated path")
axs.legend(loc='upper left')
f.show()
f.savefig('real_path_vs_simulated_path.png')

"""
# plot for real orientation vs simulated orientation
f, axs = plt.subplots(1)
time = range(len(theta_s))
time = [i * 0.1 for i in time]
end = int(len(theta_r)/len(theta_s)) * len(theta_s)

axs.plot(time[:end], theta_r[:end:int(len(theta_r)/len(theta_s))], 'c--', label="real orientation")

axs.plot(time[:end], theta_s[:end], 'm--', label="simulated orientation")

axs.set_xlabel("time (s)")
axs.set_ylabel("theta (degrees)")
axs.set_title("Real orientation vs Simulated orientation")
axs.legend(loc='upper left')
f.show()
f.savefig('real_orientation_vs_simulated_orientation.png')
"""

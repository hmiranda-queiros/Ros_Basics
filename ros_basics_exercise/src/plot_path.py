import pandas as pd
import matplotlib.pyplot as plt

headers_path = ['x', 'y', 'theta']
df_path = pd.read_csv('real_path.csv', names=headers_path)

kx = 0.35/0.22
ky = 0.2/0.13

x = df_path['x'] * kx
y = df_path['y'] * ky

headers_way = ['x', 'y']
df_way = pd.read_csv('path_waypoint.csv', names=headers_way)

x_w = df_way['x'] * kx
y_w = df_way['y'] * ky

# plot
f, axs = plt.subplots(1)
axs.plot(x, y, 'c--', label="real path")
axs.plot(x[0], y[0], 'go')
axs.annotate("Start", (x[0], y[0]), fontsize=13)

for i in range(len(x_w)):
    x_i = x_w[i]
    y_i = y_w[i]
    if i == 0:
        axs.plot(x_i, y_i, 'ro', label="way points")
        axs.annotate(i, (x_i, y_i), fontsize=14)
    else :
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
plt.show()




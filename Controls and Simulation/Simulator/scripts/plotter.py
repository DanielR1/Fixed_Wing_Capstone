### Import python packages ###
import math

import numpy as np
#import matplotlib; matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

# Update with actual file name in the data director
file_name = "data_2025-06-05_09-40-45.csv"
# Load in data as giant matrix
#data = np.loadtxt("C:/Users/ncv72/Documents/"+file_name, delimiter=',')
#close all
plt.close('all')
data = np.loadtxt("../data/"+file_name, delimiter=',')
t = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]
vx = data[:, 4]
vy = data[:, 5]
vz = data[:, 6]
# ax
# ay
# az
qw = data[:, 7]
qx = data[:, 8]
qy = data[:, 9]
qz = data[:, 10]
wx = data[:, 11]
wy = data[:, 12]
wz = data[:, 13]
f1 = data[:, 14]
f2 = data[:, 15]
f3 = data[:, 16]
f4 = data[:, 17]

# Desired trajectory
rx = data[:, 18]
ry = data[:, 19]
rz = data[:, 20]


plt.figure(1)
plt.plot(t, x, 'r', label='x')
plt.plot(t, y, 'b', label='y')
plt.plot(t, z, 'g', label='z')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Linear Position')
plt.legend()
plt.grid()


plt.figure(2)
plt.plot(t, vx, 'r')
plt.plot(t, vy, 'b')
plt.plot(t, vz, 'g')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Linear Velocity')
plt.grid()


plt.figure(3)
plt.plot(t, qw, 'r')
plt.plot(t, qx, 'b')
plt.plot(t, qy, 'g')
plt.plot(t, qz, 'y')
plt.xlabel('Time (s)')
plt.ylabel('Magnitude')
plt.title('Quaternion')
plt.grid()


plt.figure(4)
plt.plot(t, wx, 'r')
plt.plot(t, wy, 'b')
plt.plot(t, wz, 'g')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity')
plt.grid()


plt.figure(5)
plt.plot(t, f1, 'r')
plt.plot(t, f2, 'b')
plt.plot(t, f3, 'g')
plt.plot(t, f4, 'y')
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')
plt.title('Motor Forces')
plt.grid()


def quaternion_to_euler(q):
    q0, q1, q2, q3 = q

    # Roll (x-axis rotation)
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))

    # Pitch (y-axis rotation)
    pitch = math.asin(2.0 * (q0 * q2 - q3 * q1))

    # Yaw (z-axis rotation)
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    return roll, pitch, yaw



#plotting euler angles for better visualization
roll = np.zeros(len(qw))
pitch = np.zeros(len(qw))
yaw = np.zeros(len(qw))
for i in range(len(qw)):
    roll[i], pitch[i], yaw[i] = quaternion_to_euler([qw[i], qx[i], qy[i], qz[i]])

plt.figure(6)
plt.plot(t, np.rad2deg(roll), 'r', label='roll')
plt.plot(t, np.rad2deg(pitch), 'b', label='pitch')
plt.plot(t, np.rad2deg(yaw), 'g', label='yaw')
plt.xlabel('Time (s)')
plt.ylabel('Magnitude (d')
plt.title('Roll, Pitch, Yaw')
plt.grid()
plt.legend()


plt.figure(7) #desired trajectory
plt.plot(t, rx, 'r', label='rx')
plt.plot(t, ry, 'b', label='ry')
plt.plot(t, rz, 'g', label='rz')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Trajectory')
plt.legend()
plt.grid()


fig = plt.figure(8)
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='Actual Trajectory', color='blue')
ax.plot(rx, ry, rz, label='Desired Trajectory', color='red', linestyle='--')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('3D Trajectory')
ax.legend()
ax.grid(True)


## ANIMATION
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

fig = plt.figure(9)
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(np.min(x), np.max(x))
ax.set_ylim(np.min(y), np.max(y))
ax.set_zlim(np.min(z), np.max(z))
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('Animated 3D Trajectory')

actual_line, = ax.plot([], [], [], 'b-', label='Actual')
desired_line, = ax.plot([], [], [], 'r--', label='Desired')
time_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes)
ax.legend()

slice_ratio = 10
def update(frame):
    actual_line.set_data(x[::slice_ratio][:frame], y[::slice_ratio][:frame])
    actual_line.set_3d_properties(z[::slice_ratio][:frame])
    desired_line.set_data(rx[::slice_ratio][:frame], ry[::slice_ratio][:frame])
    desired_line.set_3d_properties(rz[::slice_ratio][:frame])
    time_text.set_text(f'Time = {t[::slice_ratio][frame]:.2f} s')
    return actual_line, desired_line

ani = FuncAnimation(fig, update, frames=len(t[::slice_ratio]), interval=0.5, blit=False)
plt.show()
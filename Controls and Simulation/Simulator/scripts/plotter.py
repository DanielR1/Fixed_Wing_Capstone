### Import python packages ###
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import quaternion_helpers as qhelp
import os
import glob

# Find the most recent data file in either data/ or ../data/
list_of_files = glob.glob('data/*.csv') + glob.glob('../data/*.csv')
if not list_of_files:
    raise FileNotFoundError("No data files found.")
file_name = max(list_of_files, key=os.path.getmtime)
print(f"Plotting data from: {file_name}")

plt.close('all')
data = np.loadtxt(file_name, delimiter=',')

# Row layout (NED world, FRD body) written by main.py:
# 0 t | 1-3 x,y,z (North,East,Down) | 4-6 vx,vy,vz | 7-10 qw,qx,qy,qz | 11-13 wx,wy,wz
# | 14 alpha(deg) | 15 beta(deg) | 16 T1 | 17 T2 (N) | 18 delta1 | 19 delta2 (rad)
t = data[:, 0]
x = data[:, 1]            # North
y = data[:, 2]            # East
z = data[:, 3]            # Down (NED)
altitude = -z             # up is positive
vx = data[:, 4]
vy = data[:, 5]
vz = data[:, 6]
qw = data[:, 7]
qx = data[:, 8]
qy = data[:, 9]
qz = data[:, 10]
wx = data[:, 11]
wy = data[:, 12]
wz = data[:, 13]
alpha = data[:, 14]
beta = data[:, 15]
T1 = data[:, 16]
T2 = data[:, 17]
delta1 = data[:, 18]
delta2 = data[:, 19]


# Position (NED): North, East, Altitude (= -z so "up" reads up)
plt.figure(1)
plt.plot(t, x, 'r', label='North (x)')
plt.plot(t, y, 'b', label='East (y)')
plt.plot(t, altitude, 'g', label='Altitude (-z)')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Position (NED)')
plt.legend()
plt.grid()


plt.figure(2)
plt.plot(t, vx, 'r', label='vx (North)')
plt.plot(t, vy, 'b', label='vy (East)')
plt.plot(t, vz, 'g', label='vz (Down)')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity (NED)')
plt.legend()
plt.grid()


plt.figure(3)
plt.plot(t, qw, 'r', label='qw')
plt.plot(t, qx, 'b', label='qx')
plt.plot(t, qy, 'g', label='qy')
plt.plot(t, qz, 'y', label='qz')
plt.xlabel('Time (s)')
plt.ylabel('Magnitude')
plt.title('Quaternion (body->world)')
plt.legend()
plt.grid()


plt.figure(4)
plt.plot(t, wx, 'r', label='wx (roll)')
plt.plot(t, wy, 'b', label='wy (pitch)')
plt.plot(t, wz, 'g', label='wz (yaw)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Body Angular Velocity')
plt.legend()
plt.grid()


# Motor thrusts (N)
plt.figure(5)
plt.plot(t, T1, 'r', label='T1 (left)')
plt.plot(t, T2, 'b', label='T2 (right)')
plt.xlabel('Time (s)')
plt.ylabel('Thrust (N)')
plt.title('Motor Thrusts')
plt.legend()
plt.grid()


# Elevon deflections (deg, trailing-edge-down positive)
plt.figure(6)
plt.plot(t, np.rad2deg(delta1), 'r', label='delta1 (left)')
plt.plot(t, np.rad2deg(delta2), 'b', label='delta2 (right)')
plt.xlabel('Time (s)')
plt.ylabel('Deflection (deg, TED +)')
plt.title('Elevon Deflections')
plt.legend()
plt.grid()


# Aerodynamic angles (deg)
plt.figure(7)
plt.plot(t, alpha, 'r', label='alpha (AoA)')
plt.plot(t, beta, 'b', label='beta (sideslip)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Angle of Attack / Sideslip')
plt.legend()
plt.grid()


# Euler angles (ZXY) for visualization
roll = np.zeros(len(qw))
pitch = np.zeros(len(qw))
yaw = np.zeros(len(qw))
for i in range(len(qw)):
    roll[i], pitch[i], yaw[i] = qhelp.quat_to_euler_ZXY([qw[i], qx[i], qy[i], qz[i]])
plt.figure(8)
plt.plot(t, np.rad2deg(roll), 'r', label='roll')
plt.plot(t, np.rad2deg(pitch), 'b', label='pitch')
plt.plot(t, np.rad2deg(yaw), 'g', label='yaw')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Roll, Pitch, Yaw (ZXY)')
plt.legend()
plt.grid()


# 3D trajectory (North, East, Altitude) -- altitude up
fig = plt.figure(9)
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, altitude, label='Trajectory', color='blue')
ax.set_xlabel('North (m)')
ax.set_ylabel('East (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title('3D Trajectory (NED, altitude up)')
ax.legend()
ax.grid(True)


plt.show()

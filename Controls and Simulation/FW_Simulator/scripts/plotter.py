### Import python packages ###
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)
import tkinter as tk
from tkinter import ttk
import quaternion_helpers as qhelp
import os
import glob

# Find the most recent data file in either data/ or ../data/
list_of_files = glob.glob('data/*.csv') + glob.glob('../data/*.csv')
if not list_of_files:
    raise FileNotFoundError("No data files found.")
file_name = max(list_of_files, key=os.path.getmtime)
print(f"Plotting data from: {file_name}")

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


### Tabbed-window helpers ###
# Figures are plain matplotlib Figure objects (not pyplot), collected here and shown together in
# one Tk window with one tab per figure -- like MATLAB's docked figures.
figs = []  # list of (tab title, Figure)


def new_fig(title, projection=None):
    """Create a figure + single axes, register it as a tab, and return the axes."""
    fig = Figure(figsize=(10, 6.5), layout='constrained')
    figs.append((title, fig))
    return fig.add_subplot(111, projection=projection)


def show_tabbed(figures, window_title="Simulation Plots"):
    """Show all (title, Figure) pairs in a single window with one tab each."""
    root = tk.Tk()
    root.title(window_title)
    root.geometry("1150x780")

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True)

    for title, fig in figures:
        frame = ttk.Frame(notebook)
        canvas = FigureCanvasTkAgg(fig, master=frame)
        toolbar = NavigationToolbar2Tk(canvas, frame, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        notebook.add(frame, text=title)

    # On macOS a window launched from an IDE terminal tends to open behind other apps.
    root.lift()
    root.attributes('-topmost', True)
    root.after(300, lambda: root.attributes('-topmost', False))
    root.mainloop()
    return root


def set_axes_equal(ax, xs, ys, zs, margin=0.05, min_span=0.2):
    """Equal data-unit scaling on all three axes of a 3D plot, with limits that fit the whole
    trajectory. Each axis is padded to at least `min_span` (so an almost-flat axis doesn't
    collapse), then the box aspect is set to the limit ranges -> 1 m looks the same length
    along x, y and z."""
    lo = np.array([np.min(xs), np.min(ys), np.min(zs)])
    hi = np.array([np.max(xs), np.max(ys), np.max(zs)])
    span = np.maximum(hi - lo, max(min_span, 0.1 * (hi - lo).max()))
    span = span * (1 + 2 * margin)
    mid = (lo + hi) / 2
    ax.set_xlim(mid[0] - span[0] / 2, mid[0] + span[0] / 2)
    ax.set_ylim(mid[1] - span[1] / 2, mid[1] + span[1] / 2)
    ax.set_zlim(mid[2] - span[2] / 2, mid[2] + span[2] / 2)
    ax.set_box_aspect(tuple(span))


### Plots ###
# Position (NED): North, East, Altitude (= -z so "up" reads up)
ax = new_fig('Position')
ax.plot(t, x, 'r', label='North (x)')
ax.plot(t, y, 'b', label='East (y)')
ax.plot(t, altitude, 'g', label='Altitude (-z)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (m)')
ax.set_title('Position (NED)')
ax.legend()
ax.grid(True)


ax = new_fig('Velocity')
ax.plot(t, vx, 'r', label='vx (North)')
ax.plot(t, vy, 'b', label='vy (East)')
ax.plot(t, vz, 'g', label='vz (Down)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity (m/s)')
ax.set_title('Velocity (NED)')
ax.legend()
ax.grid(True)


ax = new_fig('Quaternion')
ax.plot(t, qw, 'r', label='qw')
ax.plot(t, qx, 'b', label='qx')
ax.plot(t, qy, 'g', label='qy')
ax.plot(t, qz, 'y', label='qz')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Magnitude')
ax.set_title('Quaternion (body->world)')
ax.legend()
ax.grid(True)


ax = new_fig('Angular Rate')
ax.plot(t, wx, 'r', label='wx (roll)')
ax.plot(t, wy, 'b', label='wy (pitch)')
ax.plot(t, wz, 'g', label='wz (yaw)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angular Velocity (rad/s)')
ax.set_title('Body Angular Velocity')
ax.legend()
ax.grid(True)


# Motor thrusts (N)
ax = new_fig('Motor Thrust')
ax.plot(t, T1, 'r', label='T1 (left)')
ax.plot(t, T2, 'b', label='T2 (right)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Thrust (N)')
ax.set_title('Motor Thrusts')
ax.legend()
ax.grid(True)


# Elevon deflections (deg, trailing-edge-down positive)
ax = new_fig('Elevons')
ax.plot(t, np.rad2deg(delta1), 'r', label='delta1 (left)')
ax.plot(t, np.rad2deg(delta2), 'b', label='delta2 (right)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Deflection (deg, TED +)')
ax.set_title('Elevon Deflections')
ax.legend()
ax.grid(True)


# Aerodynamic angles (deg)
ax = new_fig('Alpha / Beta')
ax.plot(t, alpha, 'r', label='alpha (AoA)')
ax.plot(t, beta, 'b', label='beta (sideslip)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (deg)')
ax.set_title('Angle of Attack / Sideslip')
ax.legend()
ax.grid(True)


# Euler angles (ZXY) for visualization
roll = np.zeros(len(qw))
pitch = np.zeros(len(qw))
yaw = np.zeros(len(qw))
for i in range(len(qw)):
    roll[i], pitch[i], yaw[i] = qhelp.quat_to_euler_ZXY([qw[i], qx[i], qy[i], qz[i]])
ax = new_fig('Euler Angles')
ax.plot(t, np.rad2deg(roll), 'r', label='roll')
ax.plot(t, np.rad2deg(pitch), 'b', label='pitch')
ax.plot(t, np.rad2deg(yaw), 'g', label='yaw')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (deg)')
ax.set_title('Roll, Pitch, Yaw (ZXY)')
ax.legend()
ax.grid(True)


# 3D trajectory, equal scale on all axes so the path has its real-world shape.
# Plotted as East (x) / North (y) / Altitude (z): that triad is right-handed, so a left turn
# looks like a left turn. (North/East/Altitude would be left-handed, i.e. a mirror image.)
ax = new_fig('3D Trajectory', projection='3d')
ax.plot(y, x, altitude, color='blue', label='Trajectory')
ax.scatter(y[0], x[0], altitude[0], color='green', s=40, label='Start')
ax.scatter(y[-1], x[-1], altitude[-1], color='red', s=40, label='End')
set_axes_equal(ax, y, x, altitude)
for axis_name in ('x', 'y', 'z'):
    ax.locator_params(axis=axis_name, nbins=4)  # fewer ticks so narrow axes stay readable
ax.view_init(elev=22, azim=-30)
ax.set_xlabel('East (m)')
ax.set_ylabel('North (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title('3D Trajectory (equal axis scale)')
ax.legend()


show_tabbed(figs, window_title=f"Simulation Plots - {os.path.basename(file_name)}")

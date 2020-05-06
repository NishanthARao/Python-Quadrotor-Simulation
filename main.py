import time
import keyboard
import numpy as np
from quadEnv import quadrotor
import matplotlib.animation as animation
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3 

fig = plt.figure()
ax = p3.Axes3D(fig)

x_axis = np.arange(-2, 3)
y_axis = np.arange(-2, 3)
z_axis = np.arange(-2, 3)
point, = ax.plot([0], [0], [0], 'D')
ax.plot([0,0], [0,0], [0,0], 'k+')
ax.plot(x_axis, np.zeros(5), np.zeros(5), 'r--', linewidth = 0.5)
ax.plot(np.zeros(5), y_axis, np.zeros(5), 'g--', linewidth = 0.5)
ax.plot(np.zeros(5), np.zeros(5), z_axis, 'b--', linewidth = 0.5)

ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([-3, 3])

ax.set_xlabel('X-axis (in meters)')
ax.set_ylabel('Y-axis (in meters)')
ax.set_zlabel('Z-axis (in meters)')

time_display = ax.text(22.0, 1.0, 39.0, "red" ,color='red', transform=ax.transAxes)
state_display = ax.text(1.0, 1.0, 41.0, "green" ,color='green', transform=ax.transAxes)

qd = quadrotor()
qd.des_xyz(2.0, 3.0, 3.0)

def update_point(n):
    global point
    qd.PID_position()
    qd.PID_attitude()
    qd.PID_rate()
    qd.quad_motor_speed()
    state = qd.step(qd.state, qd.input_vector)
    
    point.set_data(np.array([state[9], state[10]]))
    point.set_3d_properties(state[11], 'z')
    
    time_display.set_text('Simulation time = %.1fs' % (qd.time_elapsed()))
    state_display.set_text('Position of the quad: \n x = %.1fm y = %.1fm z = %.1fm' % (qd.state[9], qd.state[10], qd.state[11]))
    
    if keyboard.is_pressed('p'):
        qd.pauseEnv()
    if keyboard.is_pressed('u'):
        qd.unpauseEnv()
    if keyboard.is_pressed('r'):
        qd.rstEnv()
    
    return point, time_display, state_display

#t1 = time.time()
ani = animation.FuncAnimation(fig, update_point, interval = 20)

plt.show()


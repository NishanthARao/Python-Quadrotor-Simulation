import time
from math import *
import keyboard
import numpy as np
from quadEnv import quadrotor
from render_animation import animate_quad
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

def rotateGFtoBF(PHI, THETA, PSI, X, Y, Z):
    X_ = cos(PSI)*cos(THETA)*X + sin(PSI)*cos(THETA)*Y - sin(THETA)*Z
    Y_ = (cos(PSI)*sin(PHI)*sin(THETA) - cos(PHI)*sin(PSI))*X + (sin(PHI)*sin(PSI)*sin(THETA)+cos(PHI)*cos(PSI))*Y + (cos(THETA)*sin(PHI))*Z
    Z_ = (cos(PHI)*cos(PSI)*sin(THETA) + sin(PHI)*sin(PSI))*X + (cos(PHI)*sin(PSI)*sin(THETA)-cos(PSI)*sin(PHI))*Y + (cos(PHI)*cos(THETA))*Z
    return (X_, Y_, Z_)
    
def rotateBFtoGF(PHI, THETA, PSI, X, Y, Z):
    X_ = cos(PSI)*cos(THETA)*X + (cos(PSI)*sin(PHI)*sin(THETA)-cos(PHI)*sin(PSI))*Y + (cos(PHI)*cos(PSI)*sin(THETA)+sin(PHI)*sin(PSI))*Z
    Y_ = sin(PSI)*cos(THETA)*X + (sin(PSI)*sin(PHI)*sin(THETA)+cos(PHI)*cos(PSI))*Y + (cos(PHI)*sin(PSI)*sin(THETA)-cos(PSI)*sin(PHI))*Z
    Z_ = -sin(THETA)*X + cos(THETA)*sin(PHI)*Y + cos(PHI)*cos(THETA)*Z
    return (X_, Y_, Z_)
    
fig = plt.figure()
ax = p3.Axes3D(fig)

x_axis = np.arange(-2, 3)
y_axis = np.arange(-2, 3)
z_axis = np.arange(-2, 3)
pointCM, = ax.plot([0], [0], [0], 'b.')
pointBLDC1, = ax.plot([0], [0], [0], 'b.')
pointBLDC2, = ax.plot([0], [0], [0], 'b.')
pointBLDC3, = ax.plot([0], [0], [0], 'b.')
pointBLDC4, = ax.plot([0], [0], [0], 'b.')
line1, = ax.plot([0,0], [0,0], [0,0], 'b.')
line2, = ax.plot([0,0], [0,0], [0,0], 'b.')

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

qd = quadrotor(Ts = 1.0/16.0, USE_PWM = 1, USE_PID = 1)
qd.des_xyz(1.0, 2.0, 3.0)

t1 = 0.0
t2 = 0.0

def update_point(n):
    global point, ax, line1, line2
    line1.remove()
    line2.remove()
    qd.PID_position()
    qd.PID_attitude()
    qd.PID_rate()
    qd.quad_motor_speed()
    state = qd.step(qd.state, qd.input_vector)
    
    #length of the arm of the quadrotor is 0.5m
    (x_bf, y_bf, z_bf) = rotateGFtoBF(state[0], state[1], state[2], state[9], state[10], state[11])
    (x_bl1, y_bl1, z_bl1) = rotateBFtoGF(state[0], state[1], state[2], x_bf, y_bf + 0.25, z_bf)
    (x_bl2, y_bl2, z_bl2) = rotateBFtoGF(state[0], state[1], state[2], x_bf + 0.25, y_bf, z_bf)
    (x_bl3, y_bl3, z_bl3) = rotateBFtoGF(state[0], state[1], state[2], x_bf, y_bf - 0.25, z_bf)
    (x_bl4, y_bl4, z_bl4) = rotateBFtoGF(state[0], state[1], state[2], x_bf - 0.25, y_bf, z_bf)
    
    #pointCM.set_data(np.array([state[9], state[10]]))
    #pointCM.set_3d_properties(state[11], 'z')
    #pointBLDC1.set_data(np.array([x_bl1, y_bl1]))
    #pointBLDC1.set_3d_properties(z_bl1, 'z')
    #pointBLDC2.set_data(np.array([x_bl2, y_bl2]))
    #pointBLDC2.set_3d_properties(z_bl2, 'z')
    #pointBLDC3.set_data(np.array([x_bl3, y_bl3]))
    #pointBLDC3.set_3d_properties(z_bl3, 'z')
    #pointBLDC4.set_data(np.array([x_bl4, y_bl4]))
    #pointBLDC4.set_3d_properties(z_bl4, 'z')
    
    line1, = ax.plot([x_bl4, x_bl2],[y_bl4,y_bl2],[z_bl4,z_bl2], 'ko-', lw=1.5, markersize=3)
    line2, = ax.plot([x_bl3,x_bl1],[y_bl3,y_bl1],[z_bl3,z_bl1], 'ko-', lw=1.5, markersize=3)
    
    #Comment this line if you don't require the trail that is left behined the quadrotor
    ax.plot([state[9]], state[10], state[11], "g.", markersize=1)
    #print qd.input_vector
    
    time_display.set_text('Simulation time = %.1fs' % (qd.time_elapsed()))
    state_display.set_text('Position of the quad: \n x = %.1fm y = %.1fm z = %.1fm' % (qd.state[9], qd.state[10], qd.state[11]))
    
    if keyboard.is_pressed('p'):
        qd.pauseEnv()
    if keyboard.is_pressed('u'):
        qd.unpauseEnv()
    if keyboard.is_pressed('r'):
        qd.rstEnv()
    
    return pointCM, pointBLDC1, pointBLDC2, pointBLDC3, pointBLDC4, time_display, state_display

#t1 = time.time()
ani = animation.FuncAnimation(fig, update_point, interval = 63)

plt.show()
'''
qd = quadrotor()
qd.des_xyz(1.0, 2.0, 3.0)

t1 = 0.0
t2 = 0.0

while True:
    t1 = time.time()
    if((t1 - t2) > qd.Ts):
        qd.PID_position()
        qd.PID_attitude()
        qd.PID_rate()
        qd.quad_motor_speed()
        state = qd.step(qd.state, qd.input_vector)
        print state[9]
        t2 = t1
'''

from math import sin, cos, acos, pi
import time
import numpy as np
from lx16a import *
import time
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button, RadioButtons



# End effector position, p6 relative to
def End_Effector_position(q, a2, d4, d6):
    q = q.reshape(6)
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    # q6 = q[5]

    p6_0 = np.array([a2 * cos(q1) * cos(q2) + d4 * cos(q1) * sin(q2 + q3) + d6 * (
                cos(q1) * (cos(q2 + q3) * cos(q4) * sin(q5) + sin(q2 + q3) * cos(q5)) + sin(q1) * sin(q4) * sin(q5)),
                     a2 * sin(q1) * cos(q2) + d4 * sin(q1) * sin(q2 + q3) + d6 * (
                                 sin(q1) * (cos(q2 + q3) * cos(q4) * sin(q5) + sin(q2 + q3) * cos(q5)) - cos(q1) * sin(
                             q4) * sin(q5)),
                     a2 * sin(q2) - d4 * cos(q2 + q3) + d6 * (
                                 sin(q2 + q3) * cos(q4) * sin(q5) - cos(q2 + q3) * cos(q5))])
    return p6_0


def Jacobian_end_effector(q, a2, d4, d6):
    q = q.reshape(6)
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    # q6 = q[5]
    J6_0 = np.array([[-d6 * (
                sin(q1) * (sin(q2 + q3) * cos(q5) + cos(q2 + q3) * cos(q4) * sin(q5)) - cos(q1) * sin(q4) * sin(
            q5)) - d4 * sin(q2 + q3) * sin(q1) - a2 * cos(q2) * sin(q1),
                      d4 * cos(q2 + q3) * cos(q1) - a2 * cos(q1) * sin(q2) + d6 * cos(q1) * (
                                  cos(q2 + q3) * cos(q5) - sin(q2 + q3) * cos(q4) * sin(q5)),
                      d4 * cos(q2 + q3) * cos(q1) + d6 * cos(q1) * (
                                  cos(q2 + q3) * cos(q5) - sin(q2 + q3) * cos(q4) * sin(q5)),
                      d6 * (cos(q4) * sin(q1) * sin(q5) - cos(q2 + q3) * cos(q1) * sin(q4) * sin(q5)),
                      -d6 * (cos(q1) * (sin(q2 + q3) * sin(q5) - cos(q2 + q3) * cos(q4) * cos(q5)) - cos(q5) * sin(
                          q1) * sin(q4)), 0],
                     [d6 * (cos(q1) * (sin(q2 + q3) * cos(q5) + cos(q2 + q3) * cos(q4) * sin(q5)) + sin(q1) * sin(
                         q4) * sin(q5)) + d4 * sin(q2 + q3) * cos(q1) + a2 * cos(q1) * cos(q2),
                      d4 * cos(q2 + q3) * sin(q1) - a2 * sin(q1) * sin(q2) + d6 * sin(q1) * (
                                  cos(q2 + q3) * cos(q5) - sin(q2 + q3) * cos(q4) * sin(q5)),
                      d4 * cos(q2 + q3) * sin(q1) + d6 * sin(q1) * (
                                  cos(q2 + q3) * cos(q5) - sin(q2 + q3) * cos(q4) * sin(q5)),
                      -d6 * (cos(q1) * cos(q4) * sin(q5) + cos(q2 + q3) * sin(q1) * sin(q4) * sin(q5)),
                      -d6 * (sin(q1) * (sin(q2 + q3) * sin(q5) - cos(q2 + q3) * cos(q4) * cos(q5)) + cos(q1) * cos(
                          q5) * sin(q4)), 0],
                     [0,
                      d4 * sin(q2 + q3) + a2 * cos(q2) + d6 * (
                                  sin(q2 + q3) * cos(q5) + cos(q2 + q3) * cos(q4) * sin(q5)),
                      d4 * sin(q2 + q3) + d6 * (sin(q2 + q3) * cos(q5) + cos(q2 + q3) * cos(q4) * sin(q5)),
                      -d6 * sin(q2 + q3) * sin(q4) * sin(q5),
                      d6 * (cos(q2 + q3) * sin(q5) + sin(q2 + q3) * cos(q4) * cos(q5)),
                      0]])
    return J6_0

def forward_kinematics(q: np.array, a2, d4, d6):
    q = q.reshape(6)
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    # Using Homogeneous_transormation_matirx
    # This is rotation of 1 relative to 0;
    A_0_1 = np.array([[cos(q1), 0, sin(q1), 0],
                      [sin(q1), 0 ,-cos(q1), 0],
                      [0      , 1,  0      , 0],
                      [0      , 0,  0      , 1]])

    A_1_2 = np.array([[cos(q2), -sin(q2), 0, a2*cos(q2)],
                      [sin(q2),  cos(q2), 0, a2*sin(q2)],
                      [0      , 0       , 1, 0],
                      [0      , 0       , 0, 1]])

    #A_2_3 = np.array([[cos(q3), -sin(q3), 0, a3*cos(q3)],
    #                  [sin(q3),  cos(q3), 0, a3*sin(q3)],
    #                  [0      , 0       , 1, 0],
    #                  [0      , 0       , 0, 1]])

    A_2_3 = np.array([[cos(q3), 0, sin(q3), 0],
                      [sin(q3), 0,-cos(q3), 0],
                      [0      , 1,  0      , 0],
                      [0      , 0,  0      , 1]])

    A_3_4 = np.array([[cos(q4), 0, sin(q4), 0],
                      [sin(q4), 0 ,-cos(q4), 0],
                      [0      ,-1,  0      , d4],
                      [0      , 0,  0      , 1]])

    A_4_5 = np.array([[cos(q5), 0, sin(q5), 0],
                      [sin(q5), 0,-cos(q5), 0],
                      [0      , 1,  0      , 0],
                      [0      , 0,  0      , 1]])

    A_5_6 = np.array([[cos(q6), -sin(q6), 0, 0],
                      [sin(q6),  cos(q6), 0, 0],
                      [0      , 0       , 1, d6],
                      [0      , 0       , 0, 1]])

    T_0_1 = A_0_1
    T_0_2 = np.matmul(A_0_1, A_1_2)
    T_0_3 = np.matmul(T_0_2, A_2_3)
    T_0_4 = np.matmul(T_0_3, A_3_4)
    T_0_5 = np.matmul(T_0_4, A_4_5)
    T_0_6 = np.matmul(T_0_5, A_5_6)

    p1 = T_0_1[0:3, 3]
    p2 = T_0_2[0:3, 3]
    p3 = T_0_3[0:3, 3]
    p4 = T_0_4[0:3, 3]
    p5 = T_0_5[0:3, 3]
    p6 = T_0_6[0:3, 3]

    p = np.array([p1, p2, p3, p4, p5, p6])

    return p

## Kinematics
# Measured distance between joint 2 and 5
measurement = 79
d4 = 160
a2 = 130
d6 = 120

# Angle between link 2 and 3
a23 = acos((pow(a2, 2)+pow(d4, 2)-pow(measurement, 2))/(2*a2*d4))

# Initial pose as defined for Inverse kinematics and initial homing position of the robot
q1 = pi/3
q2 = pi/2
q3 = -(pi/2 - a23)
q4 = 0
q5 = pi/2
q6 = 0

# q1 = 0
# q2 = 0
# q3 = 0
# q4 = 0
# q5 = 0
# q6 = 0

q0 = np.array([q1, q2, q3, q4, q5, q6]) #np.array([0, pi/2, pi/2, 0, -pi/2, 0])

points = forward_kinematics(q0,  a2, d4, d6)
p_end_effector_fk = points[5, :]
p_end_effector_ik = End_Effector_position(q0, a2, d4, d6)


fig = plt.figure()
ax = fig.gca(projection='3d')
# fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
ax.margins(x=0)

DK, = plt.plot(points[:, 0], points[:, 1], points[:, 2], label='parametric curve')

plt.xlim([-300, 300])
plt.ylim([-300, 300])
ax.set_zlim3d([-300, 300])

axcolor = 'lightgoldenrodyellow'
axq1 = plt.axes([0.25, 0.05,  0.35, 0.01], facecolor=axcolor)
axq2 = plt.axes([0.25, 0.08, 0.35, 0.01], facecolor=axcolor)
axq3 = plt.axes([0.25, 0.11, 0.35, 0.01], facecolor=axcolor)
axq4 = plt.axes([0.25, 0.14, 0.35, 0.01], facecolor=axcolor)
axq5 = plt.axes([0.25, 0.17, 0.35, 0.01], facecolor=axcolor)
axq6 = plt.axes([0.25, 0.2, 0.35, 0.01], facecolor=axcolor)

delta_q = 0.01
sq1 = Slider(axq1, 'q1', -pi/2, pi/2, valinit=q1, valstep=delta_q)
sq2 = Slider(axq2, 'q2', -pi/2, pi/2, valinit=q2, valstep=delta_q)
sq3 = Slider(axq3, 'q3', -pi/2, pi/2, valinit=q3, valstep=delta_q)
sq4 = Slider(axq4, 'q4', -pi/2, pi/2, valinit=q4, valstep=delta_q)
sq5 = Slider(axq5, 'q5', -pi/2, pi/2, valinit=q5, valstep=delta_q)
sq6 = Slider(axq6, 'q6', -pi/2, pi/2, valinit=q6, valstep=delta_q)


def update(val):
    q1 = sq1.val
    q2 = sq2.val
    q3 = sq3.val
    q4 = sq4.val
    q5 = sq5.val
    q6 = sq6.val
    q = np.array([q1, q2, q3, q4, q5, q6])
    points = forward_kinematics(q, a2, d4, d6)
    p_end_effector = End_Effector_position(q, a2, d4, d6)
    DK.set_xdata(points[:, 0])
    DK.set_ydata(points[:, 1])
    DK.set_3d_properties(points[:, 2])

    fig.canvas.draw_idle()

sq1.on_changed(update)
sq2.on_changed(update)
sq3.on_changed(update)
sq4.on_changed(update)
sq5.on_changed(update)
sq6.on_changed(update)

plt.show()

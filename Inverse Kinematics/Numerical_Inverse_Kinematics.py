from math import sin, cos, acos, pi
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# class Robot_arm:
#     def __init__(self, q0, servo_pos0):
#
# # From Joint angle to servo command
# def Joint_angle_to_servo(angle, nJoint, ):
#     # Accuracy of LX-16a is 0.24deg. That's a +1 position in the servo demand.


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


# Measured distance between joint 2 and 5
measurement = 80
d4 = 160
a2 = 130
d6 = 130

# Angle between link 2 and 3
a23 = acos((pow(a2, 2)+pow(d4, 2)-pow(measurement, 2))/(2*a2*d4))


## Inverse kinematics
# Initial pose as defined for Inverse kinematics
q1 = 0
q2 = pi/2
q3 = -(pi/2 - a23)
q4 = 0
q5 = pi/2
q6 = 0



f = 0.25
deltaT = 0.001
timeArr = np.arange(0.0, 1/f, deltaT)

# q, p, and pGoal logging
qArr = np.zeros((6, timeArr.shape[0]))
pArr = np.zeros((3, timeArr.shape[0]))
pGoalArr = np.zeros((3, timeArr.shape[0]))

# Define trajectory
q0 = np.array([0, pi/2, pi/2, 0, -pi/2, 0])
dq0 = np.zeros((1, 6))
radius = 50
pCenter = End_Effector_position(q0, a2, d4, d6)
q = q0
dq = dq0

for i in range(timeArr.shape[0]):
    t = timeArr[i]
    q = q + deltaT * dq
    qArr[:, [i]] = q.T
    pArr[:, [i]] = End_Effector_position(q, a2, d4, d6).reshape(3,1)
    pGoalArr[:, [i]] = pCenter.reshape(3, 1) + radius * np.array([sin(2*pi*f*t),0 ,cos(2*pi*f*t)]).reshape(3, 1)
    # Moore Penrose
    # numpy.linalg.pinv(a, rcond=1e-15, hermitian=False)[source]
    dq = 20 * np.dot(np.linalg.pinv( Jacobian_end_effector(q, a2, d4, d6)), (pGoalArr[:, [i]]-pArr[:, [i]])).T


fig, axs = plt.subplots(2)
axs[0].plot(pArr[0, :], pArr[2, :], 'b', pGoalArr[0, :], pGoalArr[2, :], 'r' )
axs[0].set(xlabel='X pos (mm)', ylabel='Y pos (mm)',
       title='End effector position')
axs[0].grid()

axs[1].plot(timeArr, qArr[0, :], 'y', timeArr, qArr[1, :], 'b', timeArr, qArr[2, :], 'r',
            timeArr, qArr[3, :], 'm', timeArr, qArr[4, :], 'k', timeArr, qArr[5, :], 'g')
axs[1].set(xlabel='time (mm)', ylabel='Joint angle (mm)',
       title='Servo angles')
axs[1].grid()

plt.show()

from math import sin, cos, acos, pi
import time
import numpy as np
from lx16a import *
import time
import matplotlib
import matplotlib.pyplot as plt
from cvxopt import matrix, solvers

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
measurement = 79
d4 = 160
a2 = 130
d6 = 130

# Angle between link 2 and 3
a23 = acos((pow(a2, 2)+pow(d4, 2)-pow(measurement, 2))/(2*a2*d4))

# Homing robot
#offset = Homing(servos, home)

## Inverse kinematics
# Initial pose as defined for Inverse kinematics and initial homing position of the robot
q1 = 0
q2 = pi/2
q3 = -(pi/2 - a23)
q4 = 0
q5 = pi/2
q6 = 0

f = 0.25
deltaT = 0.01
timeArr = np.arange(0.0, 1/f, deltaT)

# Define trajectory
q0 = np.array([q1, q2, q3, q4, q5, q6]) #np.array([0, pi/2, pi/2, 0, -pi/2, 0])
dq0 = np.zeros((1, 6))
radius = 50
pCenter = End_Effector_position(q0, a2, d4, d6)
pCenter[0] += 50
pCenter[2] += 50
q = q0
dq = dq0

# q, p, and pGoal logging
qArr = np.zeros((6, timeArr.shape[0]))
pArr = np.zeros((3, timeArr.shape[0]))
pGoalArr = np.zeros((3, timeArr.shape[0]))
pGoalArr[:, :] = pCenter.reshape(3, 1) + radius *np.array([np.sin(2*pi*f*timeArr), np.zeros((timeArr.shape[0])), np.cos(2*pi*f*timeArr)])

# QP problem
Q = np.identity(9)
p = np.zeros((9, 1))

for i in range(timeArr.shape[0]):
    t = timeArr[i]
    J = Jacobian_end_effector(q, a2, d4, d6)
    A = np.identity(9)
    b = np.vstack((np.transpose(np.expand_dims(q, axis=0)), np.zeros((3, 1))))
    G = np.identity(9)
    G[6, 6] = 0.0
    G[7, 7] = 0.0
    G[8, 8] = 0.0
    h = 240 * (pi/180) * np.ones((9,1)) # limit is 240 deg
    pseudo_inv_J = np.linalg.pinv(J)
    A[0, 6:9] = - 20 *deltaT* pseudo_inv_J[0, :]
    A[1, 6:9] = - 20 *deltaT* pseudo_inv_J[1, :]
    A[2, 6:9] = - 20 *deltaT* pseudo_inv_J[2, :]
    A[3, 6:9] = - 20 *deltaT* pseudo_inv_J[3, :]
    A[4, 6:9] = - 20 *deltaT* pseudo_inv_J[4, :]
    A[5, 6:9] = - 20 *deltaT* pseudo_inv_J[5, :]
    A[6, 6] = 0.0
    A[7, 7] = 0.0
    A[8, 8] = 0.0
    sol = solvers.qp(matrix(Q), matrix(p), matrix(G), matrix(h), matrix(A), matrix(b))
    q = sol['x'][1:6]
    qArr[:, [i]] = q.T
    pArr[:, [i]] = End_Effector_position(q, a2, d4, d6).reshape(3, 1)

    # Moore Penrose
    # numpy.linalg.pinv(a, rcond=1e-15, hermitian=False)[source]
    #dq = 50 * np.dot(, (pGoalArr[:, [i]]-pArr[:, [i]])).T
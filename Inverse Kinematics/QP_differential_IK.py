from math import sin, cos, acos, pi
import time
import numpy as np
from lx16a import *
import time
import matplotlib
import matplotlib.pyplot as plt
from cvxopt import matrix, solvers

# From Joint angle to servo command
# Accuracy of LX-16a is 0.24deg. That's a +1 position in the servo demand.
def Joint_angle_to_servo(angle, q0, nJoint, home):
    # Joints with belt r2/r1 = 60/30 = 2
    if nJoint == 2 or nJoint == 3:
        delta_angle = 2 * (angle - q0[nJoint - 1]) * 180/pi
    else:
        delta_angle = (angle - q0[nJoint - 1]) * 180/pi
    # If sign conventions are different
    if nJoint == 2 or nJoint == 4 or nJoint == 5 or nJoint == 6:
        servo_angle = home[nJoint - 1, 0] - delta_angle
    else:
        servo_angle = home[nJoint - 1, 0] + delta_angle
    # if servo_angle<0:
    #     servo_angle = 0
    #     print("Joint {} is below 0".format(nJoint))
    # elif servo_angle>240:
    #     servo_angle = 240
    #     print("Joint {} is above 240".format(nJoint))
    return servo_angle

# Move robot to desired position
def move_to_position(servos: list, q: np.array, q0: np.array, home: np.array):
    servo_angles = np.zeros((len(servos), 1))
    for iservo, servo in enumerate(servos):
        servo_angle = Joint_angle_to_servo(q[0, iservo], q0, iservo+1, home)
        servo_angles[iservo, 0] = servo_angle
        #servo.moveTimeWrite(angle=servo_angle, time=100)
    time.sleep(0.01)
    return servo_angles

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

# Define all the servos
#servos = [LX16A(1), LX16A(2), LX16A(3), LX16A(4), LX16A(5), LX16A(6)] #LX16A(7)
servos = [1, 2, 3, 4, 5, 6]

# Initial home position
# Define home servo angles
home = np.zeros((7, 1))
home[0] = 120
home[1] = 120  # servo position 500 = 120 deg
home[2] = 0
home[3] = 110  # servo position 458 = 110 deg
home[4] = 240
home[5] = 115
home[6] = 0

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
pCenter[0] += 0
pCenter[2] += 1
q = q0
dq = dq0

# q, p, and pGoal logging
qArr = np.zeros((6, timeArr.shape[0]))
pArr = np.zeros((3, timeArr.shape[0]))
servo_qArr = np.zeros((6, timeArr.shape[0]))
pGoalArr = np.zeros((3, timeArr.shape[0]))

# Straight line between pCenter and pGoal
pGoal = np.array([310.0, 0.0, 60])
pGoalArr[0] = pCenter[0] + (((pGoal[0] - pCenter[0])/np.amax(timeArr))*timeArr)
pGoalArr[1] = pCenter[1] + (((pGoal[1] - pCenter[1])/np.amax(timeArr))*timeArr)
pGoalArr[2] = pCenter[2] + (((pGoal[2] - pCenter[2])/np.amax(timeArr))*timeArr)

# Circle
# pGoalArr[:, :] = radius * np.array([np.sin(2*pi*f*timeArr), np.zeros((timeArr.shape[0])), np.cos(2*pi*f*timeArr)])
# pGoalArr[0, :] += pCenter[0]
# pGoalArr[1, :] += pCenter[1]
# pGoalArr[2, :] += pCenter[2] - radius - 10

# QP problem
# Implementation of this paper
# https://roam.me.columbia.edu/files/seasroamlab/publications/humanoids2013.pdf

Wx = np.identity(3)
Wdeltaq = 1 * np.identity(6)
Wq = np.identity(6)
# Q = np.identity(6)
p = np.ones((6, 1))
q_min = np.array([-120, 30, -59,  -180, 89, -180]) * pi/180
q_max = np.array([ 120, 150, 60, 180, 180, 180]) * pi/180
delta_q_min = -3 * np.ones((1, 6)) * pi/180
delta_q_max = 3 * np.ones((1, 6)) * pi/180
end_effector_error = 1e-3

qArr[:, [0]] = q.reshape(6, 1)
pArr[:, [0]] = End_Effector_position(q, a2, d4, d6).reshape(3, 1)
servo_qArr[:, [0]] = move_to_position(servos, q.reshape(1, 6), q0, home)
for i in range(1, timeArr.shape[0]):
    p_ef = End_Effector_position(q, a2, d4, d6)
    t = timeArr[i]
    J = Jacobian_end_effector(q, a2, d4, d6)
    Q1 = 2 * np.linalg.multi_dot([J.transpose(), J]) #np.linalg.multi_dot([J.transpose(), Wx, J])
    Q2 = Wdeltaq
    Q3 = Wq
    Q = Q1
    delta_p = pGoalArr[:, i] - p_ef
    dq_desired = 0.5 * (q_max - q_min) * np.ones((6, 1))
    p1 = -2 * np.linalg.multi_dot([J.transpose(), delta_p]) #- np.linalg.multi_dot([delta_p.transpose(), Wx, J, J.transpose(), Wx, J])
    p3 = - np.dot(dq_desired.transpose(), Wq)
    p = p1
    A = J
    b = (pGoalArr[:, i] - p_ef)/deltaT  # r_dot
    G = np.vstack([-np.eye(6), np.eye(6), -np.eye(6), np.eye(6)])
    lower_bound = q_min - q
    upper_bound = q_max - q
    lower_bound_delta = delta_q_min
    upper_bound_delta = delta_q_max
    h = np.hstack([-lower_bound.squeeze(), upper_bound.squeeze(), -lower_bound_delta.squeeze(), upper_bound_delta.squeeze()]).transpose()
    sol = solvers.qp(matrix(Q), matrix(p), matrix(G), matrix(h))
    q = q + np.array(sol['x']).transpose()

    servo_qArr[:, [i]] = move_to_position(servos, q, q0, home)
    time.sleep(deltaT)

    qArr[:, [i]] = q.transpose()
    pArr[:, [i]] = End_Effector_position(q, a2, d4, d6).reshape(3, 1)


fig, axs = plt.subplots(3)
axs[0].plot(pArr[0, :], pArr[2, :], 'b', pGoalArr[0, :], pGoalArr[2, :], 'r' )
axs[0].set(xlabel='X pos (mm)', ylabel='Y pos (mm)',
       title='End effector position')
axs[0].grid()

axs[1].plot(timeArr, qArr[0, :]*180/pi, 'y', timeArr, qArr[1, :]*180/pi, 'b', timeArr, qArr[2, :]*180/pi, 'r',
            timeArr, qArr[3, :]*180/pi, 'm', timeArr, qArr[4, :]*180/pi, 'k', timeArr, qArr[5, :]*180/pi, 'g')
axs[1].set(xlabel='time (s)', ylabel='Joint angle (deg)',
       title='Joint angles')
axs[1].grid()

axs[2].plot(timeArr, servo_qArr[0, :], 'y', timeArr, servo_qArr[1, :], 'b', timeArr, servo_qArr[2, :], 'r',
            timeArr, servo_qArr[3, :], 'm', timeArr, servo_qArr[4, :], 'k', timeArr, servo_qArr[5, :], 'g')
axs[2].set(xlabel='time (mm)', ylabel='servo angle (deg)',
       title='Servo angles')
axs[2].grid()

plt.show()

print('hola')
# Implementing this paper
# https://www.researchgate.net/publication/5614013_A_dual_neural_network_for_redundancy_resolution_of_kinematically_redundant_manipulators_subject_to_joint_limits_and_joint_velocity_limits

# Q = np.identity(6)
# p = np.ones((6, 1))
# q_dot_max = 1
# q_dot_min = -1
# q_max = 240 * pi/180
# q_min = 0
# k = 2
# beta = 0.5
# for i in range(timeArr.shape[0]):
#     p_ef = End_Effector_position(q, a2, d4, d6)
#     t = timeArr[i]
#     J = Jacobian_end_effector(q, a2, d4, d6)
#     A = J
#     b = (pGoalArr[:, i] - p_ef)/deltaT  # r_dot
#     G = np.vstack([ -np.eye(6), np.eye(6)])
#     lower_bound = np.vstack([q_dot_min*np.ones((1, 6)), k*(beta * q_min*np.ones((1, 6))-q)])
#     lower_bound = np.amax( lower_bound, axis=0)
#     upper_bound = np.vstack([q_dot_max*np.ones((1, 6)), k*(beta * q_max*np.ones((1, 6))-q)])
#     upper_bound = np.amin( upper_bound, axis=0)
#     h = np.hstack([-lower_bound, upper_bound])
#     sol = solvers.qp(matrix(Q), matrix(p), matrix(G), matrix(h), matrix(A), matrix(b))
#     q = np.array(sol['x'])
#     qArr[:, [i]] = q
#     pArr[:, [i]] = End_Effector_position(q, a2, d4, d6).reshape(3, 1)
#
# fig, axs = plt.subplots(2)
# axs[0].plot(pArr[0, :], pArr[2, :], 'b', pGoalArr[0, :], pGoalArr[2, :], 'r' )
# axs[0].set(xlabel='X pos (mm)', ylabel='Y pos (mm)',
#        title='End effector position')
# axs[0].grid()
#
# axs[1].plot(timeArr, qArr[0, :]*180/pi, 'y', timeArr, qArr[1, :]*180/pi, 'b', timeArr, qArr[2, :]*180/pi, 'r',
#             timeArr, qArr[3, :]*180/pi, 'm', timeArr, qArr[4, :]*180/pi, 'k', timeArr, qArr[5, :]*180/pi, 'g')
# axs[1].set(xlabel='time (mm)', ylabel='Joint angle (deg)',
#        title='Joint angles')
# axs[1].grid()

# Q = np.identity(9)
# p = np.zeros((9, 1))
# for i in range(timeArr.shape[0]):
#     p_ef = End_Effector_position(q, a2, d4, d6)
#     t = timeArr[i]
#     J = Jacobian_end_effector(q, a2, d4, d6)
#     A = np.identity(9)
#     b = np.vstack((np.transpose(np.expand_dims(q, axis=0)), np.zeros((3, 1))))
#     G = np.identity(9)
#     G[6, 6] = 0.0
#     G[7, 7] = 0.0
#     G[8, 8] = 0.0
#     h = 240 * (pi/180) * np.ones((9, 1))# limit is 240 deg
#     h[6, 0] = pGoalArr[0, i] - p_ef[0]+0.1
#     h[7, 0] = pGoalArr[1, i] - p_ef[1]+0.1
#     h[8, 0] = pGoalArr[2, i] - p_ef[2]+0.1
#     pseudo_inv_J = np.linalg.pinv(J)
#     A[0, 6:9] = - 1 *deltaT* pseudo_inv_J[0, :]
#     A[1, 6:9] = - 1 *deltaT* pseudo_inv_J[1, :]
#     A[2, 6:9] = - 1 *deltaT* pseudo_inv_J[2, :]
#     A[3, 6:9] = - 1 *deltaT* pseudo_inv_J[3, :]
#     A[4, 6:9] = - 1 *deltaT* pseudo_inv_J[4, :]
#     A[5, 6:9] = - 1 *deltaT* pseudo_inv_J[5, :]
#     A[6, 6] = 0.0
#     A[7, 7] = 0.0
#     A[8, 8] = 0.0
#     sol = solvers.qp(matrix(Q), matrix(p), matrix(G), matrix(h), matrix(A), matrix(b))
#     q = sol['x'][1:6]
#     qArr[:, [i]] = q.T
#     pArr[:, [i]] = End_Effector_position(q, a2, d4, d6).reshape(3, 1)

    # Moore Penrose
    # numpy.linalg.pinv(a, rcond=1e-15, hermitian=False)[source]
    #dq = 50 * np.dot(, (pGoalArr[:, [i]]-pArr[:, [i]])).T
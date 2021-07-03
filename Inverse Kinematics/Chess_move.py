from math import sin, cos, acos, pi
import time
import numpy as np
from lx16a import *
import time
import matplotlib
import matplotlib.pyplot as plt
from cvxopt import matrix, solvers


# Calculate servo limits
def calculate_servo_limits(servos: list, home: np.array, q0: np.array):
    servo_max = 240
    servo_min = 0
    q_servo_min = np.zeros((1, len(servos)))
    q_servo_max = np.zeros((1, len(servos)))
    q_servo_limits = np.zeros((2, len(servos)))
    for iservo, servo in enumerate(servos):
        nJoint = iservo + 1
        if nJoint == 4 or nJoint == 5 or nJoint == 6:
            q_servo_limits[0, iservo] = (- (servo_min - home[iservo]) * (pi / 180)) + q0[iservo]
            q_servo_limits[1, iservo] = (- (servo_max - home[iservo]) * (pi / 180)) + q0[iservo]
        elif nJoint == 1:
            q_servo_limits[0, iservo] = ((servo_min - home[iservo]) * (pi / 180)) + q0[iservo]
            q_servo_limits[1, iservo] = ((servo_max - home[iservo]) * (pi / 180)) + q0[iservo]
        elif nJoint == 2:
            q_servo_limits[0, iservo] = (-0.5 * (servo_min - home[iservo]) * (pi / 180)) + q0[iservo]
            q_servo_limits[1, iservo] = (-0.5 * (servo_max - home[iservo]) * (pi / 180)) + q0[iservo]
        elif nJoint == 3:
            q_servo_limits[0, iservo] = (0.5 * (servo_min - home[iservo]) * (pi / 180)) + q0[iservo]
            q_servo_limits[1, iservo] = (0.5 * (servo_max - home[iservo]) * (pi / 180)) + q0[iservo]

    q_servo_min = np.amin(q_servo_limits, axis=0)
    q_servo_max = np.amax(q_servo_limits, axis=0)
    return q_servo_min, q_servo_max

# Homing function
def Homing(servos: list, home: np.array):
    offset = np.zeros((len(servos), 1))
    for iservo, servo in enumerate(servos):
        # Move to home position
        servo.moveTimeWrite(angle=home[iservo], time=1500)
        time.sleep(0.1)
        print("Servo angle {}: {}".format(iservo, servo.getPhysicalPos()))

        # Calculate deviation
        time.sleep(0.5)
        offset[iservo] = servo.getPhysicalPos() - home[iservo]
        print("CurrentServo offset {}: {}".format(iservo, offset[iservo]))

        print("Final Servo angle {}: {}".format(iservo, servo.getPhysicalPos()))
    return offset

# From Joint angle to servo command
# Accuracy of LX-16a is 0.24deg. That's a +1 position in the servo demand.
def Joint_angle_to_servo(angle, q0, nJoint, home):
    # Joints with belt r2/r1 = 60/30 = 2
    if nJoint == 2 or nJoint == 3:
        delta_angle = 2 * (angle - q0[nJoint - 1]) * 180/pi
    else:
        delta_angle = (angle - q0[nJoint - 1]) * 180/pi
    # If sign conventions are different
    if nJoint == 2 or nJoint == 4 or nJoint == 6: #or nJoint == 6:
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
def move_to_position(servos: list, q: np.array, q0: np.array, home: np.array, b_online: bool):
    servo_angles = np.zeros((len(servos), 1))
    for iservo, servo in enumerate(servos):
        servo_angle = Joint_angle_to_servo(q[iservo], q0, iservo+1, home)
        servo_angles[iservo, 0] = servo_angle
        if b_online:
            servo.moveTimeWrite(angle=servo_angle, time=100)
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



# This is the port that the controller board is connected to
# This will be different for different computers
# On Windows, try the ports COM1, COM2, COM3, etc...
# On Raspbian, try each port in /dev/
b_robot_online = False

if b_robot_online:
    LX16A.initialize("COM3")

# Define all the servos
if b_robot_online:
    servos = [LX16A(1), LX16A(2), LX16A(3), LX16A(4), LX16A(5), LX16A(6)] #LX16A(7)
else:
    servos = [1, 2, 3, 4, 5, 6]  # dummy servos

# Initial home position
# Define home servo angles
home = np.zeros((7, 1))
home[0] = 120  # q1
home[1] = 120  # q2  # servo position 500 = 120 deg
home[2] = 55   # q3
home[3] = 110  # q4  # servo position 458 = 110 deg
home[4] = 183  # q5
home[5] = 120  # q6
home[6] = 170  # q7

# Measured distance between joint 2 and 5
measurement = 78
d4 = 160
a2 = 130
d6 = 130

# Angle between link 2 and 3, joint q3
a23 = acos((pow(a2, 2)+pow(d4, 2)-pow(measurement, 2))/(2*a2*d4))

# Homing robot
if b_robot_online:
    offset = Homing(servos, home)

## Inverse kinematics
# Initial pose as defined for Inverse kinematics and initial homing position of the robot
q1 = 0
q2 = pi/2
q3 = -(pi/2 - a23)  # Review this lower limit, it seems too low (it depends on measurement between joint 2 and 5
q4 = 0
q5 = pi/2
q6 = 0

f = 0.25
deltaT = 0.05
timeArr = np.arange(0.0, 1/f, deltaT)
n_steps = timeArr.size
n_moves = 2
n_steps_per_move = n_steps//n_moves

# Define trajectory
q0 = np.array([q1, q2, q3, q4, q5, q6]) #np.array([0, pi/2, pi/2, 0, -pi/2, 0])
dq0 = np.zeros((1, 6))
end_effector_pos = End_Effector_position(q0, a2, d4, d6)
end_effector_pos[0] += 0
end_effector_pos[2] += 0
q = q0
dq = dq0

# Solver option
solvers.options['show_progress'] = True

# q, p, and pGoal logging
qArr = np.zeros((6, timeArr.shape[0]))
pArr = np.zeros((3, timeArr.shape[0]))
dArr = np.zeros((1, timeArr.shape[0]))
servo_qArr = np.zeros((6, timeArr.shape[0]))
pGoalArr = np.zeros((3, timeArr.shape[0]))
pGoalArr1 = np.zeros((3, timeArr.shape[0]//2))
pGoalArr2 = np.zeros((3, timeArr.shape[0]//2))

# Define chess X-Y position on the chess board for a constant z coordinate

chess_pos = np.array([250.0, 50.0, -30])

pGoal1 = np.array([chess_pos[0], chess_pos[1], end_effector_pos[2]])  # Position over the piece
pGoal2 = np.array([chess_pos[0], chess_pos[1], chess_pos[2]])  # Lower end effector

pGoalArr1[0] = end_effector_pos[0] + (pGoal1[0]-end_effector_pos[0]) * (np.arange(n_steps//2)/(n_steps_per_move))
pGoalArr1[1] = end_effector_pos[1] + (pGoal1[1]-end_effector_pos[1]) * (np.arange(n_steps//2)/(n_steps_per_move))
pGoalArr1[2] = end_effector_pos[2] + (pGoal1[2]-end_effector_pos[2]) * (np.arange(n_steps//2)/(n_steps_per_move))

pGoalArr2[0] = pGoal1[0] + (pGoal2[0] - pGoal1[0]) * (np.arange(n_steps//2)/(n_steps_per_move))
pGoalArr2[1] = pGoal1[1] + (pGoal2[1] - pGoal1[1]) * (np.arange(n_steps//2)/(n_steps_per_move))
pGoalArr2[2] = pGoal1[2] + (pGoal2[2] - pGoal1[2]) * (np.arange(n_steps//2)/(n_steps_per_move))

pGoalArr = np.hstack((pGoalArr1, pGoalArr2))
# Circle
# pGoalArr[:, :] = radius * np.array([np.sin(2*pi*f*timeArr), np.zeros((timeArr.shape[0])), np.cos(2*pi*f*timeArr)])
# pGoalArr[0, :] += pCenter[0]
# pGoalArr[1, :] += pCenter[1]
# pGoalArr[2, :] += pCenter[2] - radius

# QP problem
# Or should I say SQP?

# Implementation of this paper
# https://roam.me.columbia.edu/files/seasroamlab/publications/humanoids2013.pdf

Wx = np.identity(3)
Wdeltaq = 1 * np.identity(6)
Wq = np.identity(6)
# Q = np.identity(6)
p = np.ones((6, 1))
# Physical limits of the servo (between 0 and 240 deg at servo level)
q_servo_limits = calculate_servo_limits(servos, home, q0)
q_servo_min = q_servo_limits[0]
q_servo_max = q_servo_limits[1]
# Physical limits of the robot (which should vary with the angles but at the moment defined below)
q_robot_min = np.array([-0,  30, -59,  -180, 89, -180]) * pi/180
q_robot_max = np.array([ 0, 150,  60,   180, 180, 180]) * pi/180
# Final limits
q_min = np.amax(np.vstack([q_servo_min, q_robot_min]), axis=0)
q_max = np.amin(np.vstack([q_servo_max, q_robot_max]), axis=0)
delta_q_min = -10 * np.ones((1, 6)) * pi/180
delta_q_max = 10 * np.ones((1, 6)) * pi/180
end_effector_error = 1e-3

qArr[:, [0]] = q.reshape(6, 1)
pArr[:, [0]] = End_Effector_position(q, a2, d4, d6).reshape(3, 1)
servo_qArr[:, [0]] = move_to_position(servos, q, q0, home, b_robot_online)
delta_p = pGoalArr[:, 0] - pArr[:, 0]
dArr[:, [0]] = np.linalg.norm(delta_p)

for i in range(1, timeArr.shape[0]):
    p_ef = End_Effector_position(q, a2, d4, d6)
    t = timeArr[i]
    J = Jacobian_end_effector(q, a2, d4, d6)
    Q1 = 2  * np.linalg.multi_dot([J.transpose(), J])
    #Q2 = Wdeltaq
    #Q3 = Wq
    Q = Q1
    #dq_desired = 0.5 * (q_max - q_min) * np.ones((6, 1))
    p1 = -2 * np.linalg.multi_dot([ delta_p.transpose(), J])
    #p3 = - np.dot(dq_desired.transpose(), Wq)
    p = p1
    A = J
    b = (pGoalArr[:, i] - p_ef)/deltaT  # r_dot
    G = np.vstack([-np.eye(6), np.eye(6), -np.eye(6), np.eye(6)])
    lower_bound = q_min - q
    upper_bound = q_max - q
    lower_bound_delta = delta_q_min
    upper_bound_delta = delta_q_max
    h = np.hstack([-lower_bound.squeeze(), upper_bound.squeeze(), -lower_bound_delta.squeeze(), upper_bound_delta.squeeze()]).transpose()
    opts = {'maxiters': 500, 'show_progress': False}
    sol = solvers.qp(matrix(Q), matrix(p), matrix(G), matrix(h), options = opts)
    q = q + np.array(sol['x']).transpose()

    servo_qArr[:, [i]] = move_to_position(servos, q.squeeze(), q0, home, b_robot_online)
    time.sleep(deltaT)

    qArr[:, [i]] = q.transpose()
    pArr[:, [i]] = End_Effector_position(q, a2, d4, d6).reshape(3, 1)

    delta_p = pGoalArr[:, i] - pArr[:, i]
    dArr[:, [i]] = np.linalg.norm(delta_p)
    if dArr[:, i] > 1:
        print("Too much distance {}".format(np.linalg.norm(delta_p)))
        print("iteration {}".format(i))

# Homing robot
if b_robot_online:
    offset = Homing(servos, home)

fig, axs = plt.subplots(4)
axs[0].plot(pArr[0, :], pArr[2, :], 'bo',  pGoalArr[0, :], pGoalArr[2, :], 'r' )
axs[0].set(xlabel='X pos (mm)', ylabel='Y pos (mm)',
       title='End effector position')
axs[0].grid()

axs[1].plot(timeArr, dArr[0, :], 'r')
axs[1].set(xlabel='time (s)', ylabel='Abs error (mm)',
       title='End effector error')
axs[1].grid()

axs[2].plot(timeArr, qArr[0, :]*180/pi, 'y', timeArr, qArr[1, :]*180/pi, 'b', timeArr, qArr[2, :]*180/pi, 'r',
            timeArr, qArr[3, :]*180/pi, 'm', timeArr, qArr[4, :]*180/pi, 'k', timeArr, qArr[5, :]*180/pi, 'g')
axs[2].set(xlabel='time (s)', ylabel='Joint angle (deg)',
       title='Joint angles')
axs[2].grid()

axs[3].plot(timeArr, servo_qArr[0, :], 'y', timeArr, servo_qArr[1, :], 'b', timeArr, servo_qArr[2, :], 'r',
            timeArr, servo_qArr[3, :], 'm', timeArr, servo_qArr[4, :], 'k', timeArr, servo_qArr[5, :], 'g')
axs[3].set(xlabel='time (mm)', ylabel='servo angle (deg)',
       title='Servo angles')
axs[3].grid()

plt.show()

print('hola')
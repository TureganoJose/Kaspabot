from math import sin, cos, acos, pi
import time
import numpy as np
from lx16a import *
import time
import matplotlib
import matplotlib.pyplot as plt

# class Robot_arm:
#     def __init__(self, q0, servo_pos0):
#


# Homing function
def Homing(servos: list, home: np.array):
    offset = np.zeros((len(servos), 1))
    for iservo, servo in enumerate(servos):
        # Move to home position after applying offsets
        servo.moveTimeWrite(angle=home[iservo], time=10)
        time.sleep(0.1)
        print("Servo angle {}: {}".format(iservo, servo.getPhysicalPos()))
        print("CurrentServo offset {}: {}".format(iservo, servo.angleOffsetRead()))
        # Calculate deviation
        offset[iservo] = servo.getPhysicalPos() - home[iservo]
        servo.angleOffsetAdjust(offset[iservo])
        # # Move to home position after applying offsets
        servo.moveTimeWrite(angle=home[iservo], time=200)
        time.sleep(0.5)
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
    if nJoint == 2 or nJoint == 4 or nJoint == 5 or nJoint == 6:
        servo_angle = home[nJoint - 1, 0] - delta_angle
    else:
        servo_angle = home[nJoint - 1, 0] + delta_angle
    if servo_angle<0:
        servo_angle = 0
        print("Joint {} is below 0".format(nJoint))
    elif servo_angle>240:
        servo_angle = 240
        print("Joint {} is above 240".format(nJoint))
    return servo_angle

# Move robot to desired position
def move_to_position(servos: list, q: np.array, q0: np.array, home: np.array):
    for iservo, servo in enumerate(servos):
        servo_angle = Joint_angle_to_servo(q[0, iservo], q0, iservo+1, home)
        servo.moveTimeWrite(angle=servo_angle, time=10)
        time.sleep(0.01)

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
LX16A.initialize("COM3")

# Define all the servos
servos = [LX16A(1), LX16A(2), LX16A(3), LX16A(4), LX16A(5), LX16A(6)] #LX16A(7)

# Initial home position
# Define home servo angles
home = np.zeros((7, 1))
home[0] = 120
home[1] = 120
home[2] = 0
home[3] = 110
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
offset = Homing(servos, home)

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

# q, p, and pGoal logging
qArr = np.zeros((6, timeArr.shape[0]))
pArr = np.zeros((3, timeArr.shape[0]))
pGoalArr = np.zeros((3, timeArr.shape[0]))

# Define trajectory
q0 = np.array([q1, q2, q3, q4, q5, q6]) #np.array([0, pi/2, pi/2, 0, -pi/2, 0])
dq0 = np.zeros((1, 6))
radius = 50
pCenter = End_Effector_position(q0, a2, d4, d6)
pCenter[0] += 100
pCenter[2] += 100
q = q0
dq = dq0

for i in range(timeArr.shape[0]):
    t = timeArr[i]
    q = q + deltaT * dq
    move_to_position(servos, q, q0, home)
    time.sleep(deltaT)
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
LX16A.moveStopAll()
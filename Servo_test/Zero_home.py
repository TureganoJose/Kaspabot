from lx16a import *
from math import sin, cos, acos, pi
import time

# This is the port that the controller board is connected to
# This will be different for different computers
# On Windows, try the ports COM1, COM2, COM3, etc...
# On Raspbian, try each port in /dev/
LX16A.initialize("COM3")

# There should two servos connected, with IDs 1 and 2
servo1 = LX16A(1)
servo2 = LX16A(2)
servo3 = LX16A(3)
servo4 = LX16A(4)
servo5 = LX16A(5)
servo6 = LX16A(6)
servo7 = LX16A(7)

# Define home servo angles
home1 = 120
home2 = 120
home3 = 0
home4 = 110
home5 = 240
home6 = 120
home7 = 0


# Move to home position after applying offsets
servo1.moveTimeWrite( angle=home1, time=1000)
time.sleep(2)
print("Servo angle 1 %lf" % servo1.getPhysicalPos())
servo2.moveTimeWrite( angle=home2, time=1000)
time.sleep(2)
print("Servo angle 2 %lf" % servo2.getPhysicalPos())
servo3.moveTimeWrite( angle=home3, time=1000)
time.sleep(2)
print("Servo angle 3 %lf" % servo3.getPhysicalPos())
servo4.moveTimeWrite( angle=home4, time=1000)
time.sleep(2)
print("Servo angle 4 %lf" % servo4.getPhysicalPos())
servo5.moveTimeWrite( angle=home5, time=1000)
time.sleep(2)
print("Servo angle 5 %lf" % servo5.getPhysicalPos())
servo6.moveTimeWrite( angle=home6, time=1000)
time.sleep(2)
print("Servo angle 6 %lf" % servo6.getPhysicalPos())
# servo7.moveTimeWrite( relAngle=0.5, time=20000)
print("Servo angle 7 %lf" % servo7.angle)

# Check all the offsets are set to 0
print("Servo offset 1 %lf" % servo1.angleOffsetRead())
print("Servo offset 2 %lf" % servo2.angleOffsetRead())
print("Servo offset 3 %lf" % servo3.angleOffsetRead())
print("Servo offset 4 %lf" % servo4.angleOffsetRead())
print("Servo offset 5 %lf" % servo5.angleOffsetRead())
print("Servo offset 6 %lf" % servo6.angleOffsetRead())
print("Servo offset 7 %lf" % servo7.angleOffsetRead())


# Calculate deviation

offset1 = servo1.getPhysicalPos() - home1
servo1.angleOffsetAdjust(offset1)
servo1.angleOffsetWrite()
offset2 = servo2.getPhysicalPos() - home2
servo2.angleOffsetAdjust(offset2)
servo2.angleOffsetWrite()
offset3 = servo3.getPhysicalPos() - home3
servo3.angleOffsetAdjust(offset3)
servo3.angleOffsetWrite()
offset4 = servo4.getPhysicalPos() - home4
servo4.angleOffsetAdjust(offset4)
servo4.angleOffsetWrite()
offset5 = servo5.getPhysicalPos() - home5
servo5.angleOffsetAdjust(offset5)
servo5.angleOffsetWrite()
offset6 = servo6.getPhysicalPos() - home6
servo6.angleOffsetAdjust(offset6)
servo6.angleOffsetWrite()
# offset7 = servo6.getPhysicalPos() - home7
# servo7.angleOffsetAdjust(offset6)


# print("Servo angle 1 %lf" % servo1.getPhysicalPos())
# print("Servo angle 2 %lf" % servo2.getPhysicalPos())
# print("Servo angle 3 %lf" % servo3.getPhysicalPos())
# print("Servo angle 4 %lf" % servo4.getPhysicalPos())
# print("Servo angle 5 %lf" % servo5.getPhysicalPos())
# print("Servo angle 6 %lf" % servo6.getPhysicalPos())
# print("Servo angle 7 %lf" % servo7.getPhysicalPos())


# # Move to home position after applying offsets
servo1.moveTimeWrite( angle=home1, time=2000)
time.sleep(2)
print("Servo angle 1 %lf" % servo1.getPhysicalPos())
servo2.moveTimeWrite( angle=home2, time=2000)
time.sleep(2)
print("Servo angle 2 %lf" % servo2.getPhysicalPos())
servo3.moveTimeWrite( angle=home3, time=2000)
time.sleep(2)
print("Servo angle 3 %lf" % servo3.getPhysicalPos())
servo4.moveTimeWrite( angle=home4, time=2000)
time.sleep(2)
print("Servo angle 4 %lf" % servo4.getPhysicalPos())
servo5.moveTimeWrite( angle=home5, time=2000)
time.sleep(2)
print("Servo angle 5 %lf" % servo5.getPhysicalPos())
servo6.moveTimeWrite( angle=home6, time=20000)
time.sleep(2)
print("Servo angle 6 %lf" % servo6.getPhysicalPos())
# servo7.moveTimeWrite( relAngle=0.5, time=20000)
print("Servo angle 7 %lf" % servo7.angle)


# Measured distance between joint 2 and 5
measurement = 82.13
d4 = 160
a2 = 130
c2 = (pow(a2, 2)-pow(measurement, 2) - pow(d4, 2))/(2*d4)
c1 = 160 - c2

# Angle between link 2 and 3
a23 = acos((pow(a2, 2)+pow(d4, 2)-pow(measurement, 2))/(2*a2*d4))

# Initial pose
q1 = 0
q2 = pi/2
q3 = -(pi/2 - a23)
q4 = 0
q5 = pi/2
q6 = 0

LX16A.moveStopAll()
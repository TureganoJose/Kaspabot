from lx16a import *
from math import sin, cos

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

print("Servo angle 1 %lf" % servo1.angle)
print("Servo angle 2 %lf" % servo2.angle)
print("Servo angle 3 %lf" % servo3.angle)
print("Servo angle 4 %lf" % servo4.angle)
print("Servo angle 5 %lf" % servo5.angle)
print("Servo angle 6 %lf" % servo6.angle)
print("Servo angle 7 %lf" % servo7.angle)

home1 = 120
home2 = 120
home3 = 0
home4 = 120
home5 = 240
home6 = 120
home7 = 0


servo1.moveTimeWrite( angle=home1, time=20000)
print("Servo angle 1 %lf" % servo1.getPhysicalPos())
servo2.moveTimeWrite( angle=home2, time=20000)
print("Servo angle 2 %lf" % servo2.getPhysicalPos())
servo3.moveTimeWrite( angle=home3, time=20000)
print("Servo angle 3 %lf" % servo3.getPhysicalPos())
servo4.moveTimeWrite( angle=home4, time=20000)
print("Servo angle 4 %lf" % servo4.getPhysicalPos())
servo5.moveTimeWrite( angle=home5, time=20000)
print("Servo angle 5 %lf" % servo5.getPhysicalPos())
servo6.moveTimeWrite( angle=home6, time=20000)
print("Servo angle 6 %lf" % servo6.getPhysicalPos())
# servo7.moveTimeWrite( relAngle=0.5, time=20000)
print("Servo angle 7 %lf" % servo7.angle)
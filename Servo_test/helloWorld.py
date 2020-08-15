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


#print("The servo 1 is supposed to be at position", servo1.getVirtualPos())
print("The servo 1 is physically at position", servo2.getPhysicalPos())
print("The servo 1 offset is", servo2.angleOffsetRead())
servo2.angleOffsetAdjust(12)
servo2.angleOffsetWrite()
print("The servo 1 is physically at position", servo2.getPhysicalPos())
print("The servo 1 offset is", servo2.angleOffsetRead())

print("The servo 1 offset is", servo2.angleOffsetRead())
print("The servo 1 upper limit", servo2.upperLimit)
print("The servo 1 lower limit", servo2.lowerLimit)

print("Servo angle 1 %lf" % servo1.angle)
servo1.moveTimeWrite(120, 10000)

print("Servo angle 1 %lf" % servo1.angle)
print("The servo 1 is supposed to be at position", servo1.getVirtualPos())
print("The servo 1 is physically at position", servo1.getPhysicalPos())


print("Servo angle 2 %lf" % servo1.angle)
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

deviation = servo1.angle

servo1.moveTimeWriteRel( relAngle=0.5, time=20000)
print("Servo angle 1 %lf" % servo1.angle)
servo2.moveTimeWriteRel( relAngle=0.5, time=20000)
print("Servo angle 2 %lf" % servo2.angle)
servo3.moveTimeWriteRel( relAngle=0.5, time=20000)
print("Servo angle 3 %lf" % servo3.angle)
servo4.moveTimeWriteRel( relAngle=0.5, time=20000)
print("Servo angle 4 %lf" % servo4.angle)
servo5.moveTimeWriteRel( relAngle=0.5, time=20000)
print("Servo angle 5 %lf" % servo5.angle)
servo6.moveTimeWriteRel( relAngle=0.5, time=20000)
print("Servo angle 6 %lf" % servo6.angle)
servo7.moveTimeWriteRel( relAngle=0.5, time=20000)
print("Servo angle 7 %lf" % servo7.angle)

# t = 0

# while True:
# 	# Two sine waves out of phase
# 	# The servos can rotate between 0 and 240 degrees,
# 	# So we adjust the waves to be in that range
#
# 	# servo1.moveTimeWrite(sin(t) * 120 + 120)
# 	servo_mode  = servo1.servoMotorModeRead() # 0-> servo mode 1 -> motor mode
#
# 	#print("Servo1 mode"+ servo_mode[0])
#
# 	print(servo1.angle)
# 	print(servo2.angle)
# 	print(servo3.angle)
# 	print(servo4.angle)
# 	print(servo5.angle)
# 	print(servo6.angle)
# 	print(servo7.angle)
# 	#servo1.moveTimeWrite( angle=0, time=10000)
#
#
# 	print("Servo 1 pos %d" % servo1.getPhysicalPos())
# 	print("Anles offset %d" % servo1.angleOffsetRead() )
# 	#servo2.moveTimeWrite(cos(t) * 120 + 120)
# 	#print("Servo 2 pos %d" % servo2.getPhysicalPos())
# 	t += 0.01

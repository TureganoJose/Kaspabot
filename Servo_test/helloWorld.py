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

servo1.moveTimeWrite( angle=0, time=20000)
print(servo1.angle)
servo2.moveTimeWrite( angle=20, time=20000)
print(servo2.angle)
servo3.moveTimeWrite( angle=30, time=20000)
print(servo3.angle)
servo4.moveTimeWrite( angle=40, time=20000)
print(servo4.angle)
servo5.moveTimeWrite( angle=0, time=20000)
print(servo5.angle)
servo6.moveTimeWrite( angle=0, time=20000)
print(servo6.angle)
servo7.moveTimeWrite( angle=0, time=20000)
print(servo7.angle)

t = 0

while True:
	# Two sine waves out of phase
	# The servos can rotate between 0 and 240 degrees,
	# So we adjust the waves to be in that range

	# servo1.moveTimeWrite(sin(t) * 120 + 120)
	servo_mode  = servo1.servoMotorModeRead() # 0-> servo mode 1 -> motor mode

	#print("Servo1 mode"+ servo_mode[0])

	print(servo1.angle)
	print(servo2.angle)
	print(servo3.angle)
	print(servo4.angle)
	print(servo5.angle)
	print(servo6.angle)
	print(servo7.angle)
	#servo1.moveTimeWrite( angle=0, time=10000)


	print("Servo 1 pos %d" % servo1.getPhysicalPos())
	print("Anles offset %d" % servo1.angleOffsetRead() )
	#servo2.moveTimeWrite(cos(t) * 120 + 120)
	#print("Servo 2 pos %d" % servo2.getPhysicalPos())
	t += 0.01

import numpy as np
from math import pi, sqrt, atan2, sin, cos, pow

# Resources:
# https://www.uio.no/studier/emner/matnat/ifi/INF3480/v14/undervisningsmatriale/lec05-Inverse-VelocityKinematicsI.pdf
# Book: 	Villani L., Oriolo G., Siciliano B.: Robotics: Modelling, Planning and Control,  Page 78


def ik(K: np.array):
    # Anthropomorphic arm with 6 DOF and spherical wrist
    # It calculates the Inverse Kinematic of an Anthropomorphic arm with 6 DOF.
    # 'q' is the solutions in radiant and K is the direct Kinematic matrix.
    #
    #               K = [ n s a p;
    #                     0 0 0 1]
    # where n, s, a are three vectors fo 3 elements that represents the
    # end-effector's orientation, and p is the desired end-effector position.
    # Denavit-Hartenberg's Parameters
    a1 = 0           #[m]
    a2 = 0.13         #[m]
    a3 = 0.16         #[m] d4 forearm
    d6 = 0.13        #[m]


    # dk=[n s a p; 0 0 0 1]
    # n, s, a: They are 3 vector for the end-effector's orientation
    dk = K          # Position and orientation of end-effector
    n = np.array(dk[0:3, 0])
    s = np.array(dk[0:3, 1])
    a = np.array(dk[0:3, 2])
    R = np.array([n, s, a])
    dk = K          # Direct kinematics matrix
    # Inverse Kinematic
    p_ot = dk[0:3, 3] # End-effector's position
    pw = p_ot-d6*n   # Wrist's position
    pw_x = pw[0]   # Vector's components that represents the wrist's position
    pw_y = pw[1]
    pw_z = pw[2]
    c3 = (pow(pw_x, 2)+pow(pw_y, 2)+pow(pw_z, 2)-pow(a2, 2)-pow(a3, 2))/(2*a2*a3)  # cos(teta3)
    s3 = -sqrt(1-pow(c3, 2))        # sin(teta3)
    teta3 = atan2(s3,c3)
    c2 = (sqrt(pow(pw_x,2)+pow(pw_y,2))*(a2+a3*c3)+pw_z*a3*s3)/(pow(a2,2)+pow(a3,2)+2*a2*a3*c3)      # cos(teta2)
    s2 = (pw_z*(a2+a3*c3)-sqrt(pow(pw_x,2)+pow(pw_y,2))*a3*s3)/(pow(a2,2)+pow(a3,2)+2*a2*a3*c3)     # sin(teta2)
    teta2 = atan2((a2+a3*c3)*pw_z-a3*s3*sqrt(pow(pw_x,2)+pow(pw_y,2)),(a2+a3*c3)*sqrt(pow(pw_x,2)+pow(pw_y,2))+a3*s3*pw_z)
    teta1 = atan2(pw_y, pw_x)
    R3_0 = np.array([[cos(teta1)*cos(teta2+teta3), -cos(teta1)*sin(teta2+teta3), sin(teta1)],      # Matrix for the Euler's angle of 3dof arm
        [sin(teta1)*cos(teta2+teta3), -sin(teta1)*sin(teta2+teta3), -cos(teta1)],
        [sin(teta2+teta3), cos(teta2+teta3), 0]])
    R6_3 = np.multiply(R3_0.T, R)        # Matrix for the Eulers angle of spherical wrist
    # Inverse kinematic for the spherical wrist
    teta4 = atan2(R6_3[1, 2], R6_3[0, 2])
    teta5 = atan2(sqrt(pow((R6_3[0, 2]),2)+pow((R6_3[1, 2]), 2)), R6_3[2, 2])
    teta6 = atan2(R6_3[2, 1], R6_3[2, 0])
    q = np.array([teta1, teta2, teta3, teta4, teta5, teta6])      # Solutions in radiant
    return q

def ik_2(R: np.array):
    # Source https://www.youtube.com/watch?v=D93iQVoSScQ
    q1 = atan2(pw_y, pw_x)
    r1 = sqrt(pow(pw_x,2)+pow(pw_y,2))
    r2 = pw_z-a1
    r3 = sqrt(pow(r1,2)+pow(r2,2))
    ph2 = atan2(r2,r1)
    ph1 = acos((pow(a3,2)-pow(a2,2)-pow(r3,2))/(-2*a2*r3))
    q2 = ph2 - ph1
    phi3 = acos((pow(r3,2)-pow(a2,2)-pow(a3,2))/(-2*a2*a3))
    q3 = pi/2 -phi3
    #  For the DH parameters, we can derive R3_0 from the forward kinematics
    # Rotation of 3 relative to 0: R3_0
    R3_0 = np.array([[cos(q1)*cos(q2+q3), -cos(q1)*sin(q2+q3), sin(q1)],      # Matrix for the Euler's angle of 3dof arm
        [sin(q1)*cos(q2+q3), -sin(q1)*sin(q2+q3), -cos(q1)],
        [sin(q2+q3), cos(q2+q3), 0]])
    # Given orientation of EE
    R6_0 = R
    inv_R3_0 = np.linalg.inv(R3_0)
    R6_3 = np.dot(inv_R3_0, R6_0)

    q5 = np.arccos(R6_3[2][2])
    q6 = np.arccos(-R6_3[2][0]/np.sin(q5))
    q4 = np.arccos(R6_3[1][2]/np.sin(q5))

    q = np.array([q1, q2, q3, q4, q5, q6])
    return q

# Euler angle, rotation z-y-x
# https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
# X phi
# Y theta
# Z psi
phi = -180*pi/180 #z
theta = 90*pi/180 #y # Home position distance between 2 and 5 = 60mm angle = -20.756 *pi/180
psi = 0*pi/180 # x

R = np.array([[cos(theta)*cos(psi), -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)],
             [cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)],
             [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]])



# https://www.continuummechanics.org/rotationmatrix.html
# Identity matrix matches orientation of frame 0
# Rotation ZYZ
z = 0*pi/180
y = 180*pi/180
z_2 = 0*pi/180
R_2 = np.array([[cos(z)*cos(y)*cos(z_2)-sin(z)*sin(z_2),-cos(z)*cos(y)*sin(z_2)-sin(z)*cos(z_2),cos(z)*sin(y)],
             [sin(z)*cos(y)*cos(z_2)+cos(z)*sin(z_2), -sin(z)*cos(y)*sin(z_2)+cos(z)*cos(z_2),sin(z)*sin(y)],
             [-sin(y)*cos(z_2),                       sin(y)*sin(z_2),                        cos(y)]])


p = np.array([[0.1769],[0],[0.0241]])
#p = np.array([[0.17824],[0],[0.0264]])
#p = np.array([[0.29],[0],[0.13]])

K = np.append(R, p, 1)
k = np.vstack((K, [0, 0, 0, 1]))

theta = ik(k)






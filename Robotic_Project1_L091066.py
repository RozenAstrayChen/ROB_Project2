from math import *
import numpy as np
import matplotlib.pyplot as plt # show the picure
from mpl_toolkits.mplot3d import Axes3D # show 3D picture
from matplotlib.patches import FancyArrowPatch # draw a vector
from mpl_toolkits.mplot3d import proj3d # draw 3D vector
np.set_printoptions(suppress=True)

T=np.zeros((4,4))
A = np.zeros((4, 4, 6))

def task1(theta1,theta2,theta3,theta4,theta5,theta6):
    ap = np.array([(-90 / 180 * pi), 0, (90 / 180 * pi), (-90 / 180 * pi), (90 / 180 * pi), 0])
    a = np.array([0, 0.432, -0.02, 0, 0, 0])
    d = np.array([0, 0, 0.149, 0.433, 0, 0])
    # theta = [(rand*2-1)*160 /180*pi,(rand*2-1)*125 /180*pi,(rand*2-1)*135 /180*pi,(rand*2-1)*140 /180*pi,(rand*2-1)*100 /180*pi,(rand*2-1)*260 /180*pi]
    theta = [theta1 / 180 * pi, theta2 / 180 * pi, theta3 / 180 * pi, theta4 / 180 * pi, theta5 / 180 * pi,
             theta6 / 180 * pi]
    T = np.zeros((4, 4))
    A = np.zeros((4, 4, 6))
    # calculate A1~A6
    for i in range(6):
        A[:, :, i] = [[cos(theta[i]), -sin(theta[i]) * cos(ap[i]), sin(theta[i]) * sin(ap[i]), a[i] * cos(theta[i])]
            , [sin(theta[i]), cos(theta[i]) * cos(ap[i]), -cos(theta[i]) * sin(ap[i]), a[i] * sin(theta[i])]
            , [0, sin(ap[i]), cos(ap[i]), d[i]]
            , [0, 0, 0, 1]]
    # calculate T=A1*A2*A3*A4*A5*A6
    T = np.matmul(
        np.matmul(np.matmul(np.matmul(np.matmul(A[:, :, 0], A[:, :, 1]), A[:, :, 2]), A[:, :, 3]), A[:, :, 4]),
        A[:, :, 5])
    #print('T=\n',T)
    x = T[0,3]
    y = T[1,3]
    z = T[2,3]
    phi = atan2(T[1,2],T[0,2]) /pi*180
    theta = atan2(((T[0,2]**2+T[1,2]**2)**0.5),T[2,2])/ pi*180
    psi = atan2(T[2,1],-T[2,0]) / pi*180
    #print("(x, y, z) = \n",x,y,z)
    #print("(phi, theta, psi) =\n",phi,theta,psi)
    xyz=[x,y,z]
    position=[phi,theta,psi]
    A=T
    return A,xyz,position

def task2(T):
    #print('T=',T)
    d3 = 0.149
    d4 = 0.433
    a2 = 0.432
    a3 = -0.02
    px = T[0, 3]/100  #cm to m
    ax = T[0, 2]
    ox = T[0, 1]
    nx = T[0, 0]
    py = T[1, 3]/100
    ay = T[1, 2]
    oy = T[1, 1]
    ny = T[1, 0]
    pz = T[2, 3]/100
    az = T[2, 2]
    oz = T[2, 1]
    nz = T[2, 0]
    p1 = (px ** 2 + py ** 2) ** 0.5
    p3 = (d4 ** 2 + a3 ** 2) ** 0.5
    M = (px ** 2 + py ** 2 + pz ** 2 - a2 ** 2 - a3 ** 2 - d4 ** 2 - d3 ** 2) / (2 * a2)

    theta1_1 = (atan2(py, px) - atan2(d3 / p1, (1 - (d3 / p1) ** 2) ** 0.5))
    theta1_2 = (atan2(py, px) - atan2(d3 / p1, -(1 - (d3 / p1) ** 2) ** 0.5))
    theta3_1 = (-atan2(a3, d4) + atan2(M / p3, (1 - (M / p3) ** 2) ** 0.5))
    theta3_2 = (-atan2(a3, d4) + atan2(M / p3, -(1 - (M / p3) ** 2) ** 0.5))
    theta2_1 = atan2(-(a3 + a2 * cos(theta3_1)) * pz + (cos(theta1_1) * px + sin(theta1_1) * py) * (a2 * sin(theta3_1) + d4), (d4 + a2 * sin(theta3_1)) * pz + (cos(theta1_1) * px + sin(theta1_1) * py) * (a2 * cos(theta3_1) + a3)) - theta3_1
    theta2_2 = atan2(-(a3 + a2 * cos(theta3_2)) * pz + (cos(theta1_1) * px + sin(theta1_1) * py) * (a2 * sin(theta3_2) + d4), (d4 + a2 * sin(theta3_2)) * pz + (cos(theta1_1) * px + sin(theta1_1) * py) * (a2 * cos(theta3_2) + a3)) - theta3_2
    theta2_3 = atan2(-(a3 + a2 * cos(theta3_1)) * pz + (cos(theta1_2) * px + sin(theta1_2) * py) * (a2 * sin(theta3_1) + d4), (d4 + a2 * sin(theta3_1)) * pz + (cos(theta1_2) * px + sin(theta1_2) * py) * (a2 * cos(theta3_1) + a3)) - theta3_1
    theta2_4 = atan2(-(a3 + a2 * cos(theta3_2)) * pz + (cos(theta1_2) * px + sin(theta1_2) * py) * (a2 * sin(theta3_2) + d4), (d4 + a2 * sin(theta3_2)) * pz + (cos(theta1_2) * px + sin(theta1_2) * py) * (a2 * cos(theta3_2) + a3)) - theta3_2

    theta4_1 = atan2(-ax * sin(theta1_1) + cos(theta1_1) * ay, ax * cos(theta1_1) * cos(theta2_1 + theta3_1) + sin(theta1_1) * cos(theta2_1 + theta3_1) * ay - sin(theta2_1 + theta3_1) * az)
    theta4_2 = atan2(ax * sin(theta1_1) - cos(theta1_1) * ay, -ax * cos(theta1_1) * cos(theta2_1 + theta3_1) - sin(theta1_1) * cos(theta2_1 + theta3_1) * ay + sin(theta2_1 + theta3_1) * az)
    theta4_3 = atan2(-ax * sin(theta1_1) + cos(theta1_1) * ay, ax * cos(theta1_1) * cos(theta2_2 + theta3_2) + sin(theta1_1) * cos(theta2_2 + theta3_2) * ay - sin(theta2_2 + theta3_2) * az)
    theta4_4 = atan2(ax * sin(theta1_1) - cos(theta1_1) * ay, -ax * cos(theta1_1) * cos(theta2_2 + theta3_2) - sin(theta1_1) * cos(theta2_2 + theta3_2) * ay + sin(theta2_2 + theta3_2) * az)
    theta4_5 = atan2(-ax * sin(theta1_2) + cos(theta1_2) * ay, ax * cos(theta1_2) * cos(theta2_3 + theta3_1) + sin(theta1_2) * cos(theta2_3 + theta3_1) * ay - sin(theta2_3 + theta3_1) * az)
    theta4_6 = atan2(ax * sin(theta1_2) - cos(theta1_2) * ay, -ax * cos(theta1_2) * cos(theta2_3 + theta3_1) - sin(theta1_2) * cos(theta2_3 + theta3_1) * ay + sin(theta2_3 + theta3_1) * az)
    theta4_7 = atan2(-ax * sin(theta1_2) + cos(theta1_2) * ay, ax * cos(theta1_2) * cos(theta2_4 + theta3_2) + sin(theta1_2) * cos(theta2_4 + theta3_2) * ay - sin(theta2_4 + theta3_2) * az)
    theta4_8 = atan2(ax * sin(theta1_2) - cos(theta1_2) * ay, -ax * cos(theta1_2) * cos(theta2_4 + theta3_2) - sin(theta1_2) * cos(theta2_4 + theta3_2) * ay + sin(theta2_4 + theta3_2) * az)

    theta6_1 = atan2(-cos(theta1_1) * sin(theta2_1 + theta3_1) * ox - sin(theta1_1) * sin(theta2_1 + theta3_1) * oy - cos(theta2_1 + theta3_1) * oz,  cos(theta1_1) * sin(theta2_1 + theta3_1) * nx + sin(theta1_1) * sin(theta2_1 + theta3_1) * ny + cos(theta2_1 + theta3_1) * nz)
    theta6_2 = atan2(cos(theta1_1)  * sin(theta2_1 + theta3_1) * ox + sin(theta1_1) * sin(theta2_1 + theta3_1) * oy + cos(theta2_1 + theta3_1) * oz, -cos(theta1_1) * sin(theta2_1 + theta3_1) * nx - sin(theta1_1) * sin(theta2_1 + theta3_1) * ny - cos(theta2_1 + theta3_1) * nz)
    theta6_3 = atan2(-cos(theta1_1) * sin(theta2_2 + theta3_2) * ox - sin(theta1_1) * sin(theta2_2 + theta3_2) * oy - cos(theta2_2 + theta3_2) * oz,  cos(theta1_1) * sin(theta2_2 + theta3_2) * nx + sin(theta1_1) * sin(theta2_2 + theta3_2) * ny + cos(theta2_2 + theta3_2) * nz)
    theta6_4 = atan2(cos(theta1_1)  * sin(theta2_2 + theta3_2) * ox + sin(theta1_1) * sin(theta2_2 + theta3_2) * oy + cos(theta2_2 + theta3_2) * oz, -cos(theta1_1) * sin(theta2_2 + theta3_2) * nx - sin(theta1_1) * sin(theta2_2 + theta3_2) * ny - cos(theta2_2 + theta3_2) * nz)
    theta6_5 = atan2(-cos(theta1_2) * sin(theta2_3 + theta3_1) * ox - sin(theta1_2) * sin(theta2_3 + theta3_1) * oy - cos(theta2_3 + theta3_1) * oz,  cos(theta1_2) * sin(theta2_3 + theta3_1) * nx + sin(theta1_2) * sin(theta2_3 + theta3_1) * ny + cos(theta2_3 + theta3_1) * nz)
    theta6_6 = atan2(cos(theta1_2)  * sin(theta2_3 + theta3_1) * ox + sin(theta1_2) * sin(theta2_3 + theta3_1) * oy + cos(theta2_3 + theta3_1) * oz, -cos(theta1_2) * sin(theta2_3 + theta3_1) * nx - sin(theta1_2) * sin(theta2_3 + theta3_1) * ny - cos(theta2_3 + theta3_1) * nz)
    theta6_7 = atan2(-cos(theta1_2) * sin(theta2_4 + theta3_2) * ox - sin(theta1_2) * sin(theta2_4 + theta3_2) * oy - cos(theta2_4 + theta3_2) * oz,  cos(theta1_2) * sin(theta2_4 + theta3_2) * nx + sin(theta1_2) * sin(theta2_4 + theta3_2) * ny + cos(theta2_4 + theta3_2) * nz)
    theta6_8 = atan2(cos(theta1_2)  * sin(theta2_4 + theta3_2) * ox + sin(theta1_2) * sin(theta2_4 + theta3_2) * oy + cos(theta2_4 + theta3_2) * oz, -cos(theta1_2) * sin(theta2_4 + theta3_2) * nx - sin(theta1_2) * sin(theta2_4 + theta3_2) * ny - cos(theta2_4 + theta3_2) * nz)

    theta5_1 = atan2((cos(theta1_1) * cos(theta4_1) * cos(theta2_1 + theta3_1) - sin(theta1_1) * sin(theta4_1)) * ax + (cos(theta1_1) * sin(theta4_1) + cos(theta4_1) * cos(theta2_1 + theta3_1) * sin(theta1_1)) * ay - cos(theta4_1) * sin(theta2_1 + theta3_1) * az, cos(theta1_1) * sin(theta2_1 + theta3_1) * ax + sin(theta1_1) * sin(theta2_1 + theta3_1) * ay + cos(theta2_1 + theta3_1) * az)
    theta5_2 = atan2((cos(theta1_1) * cos(theta4_2) * cos(theta2_1 + theta3_1) - sin(theta1_1) * sin(theta4_2)) * ax + (cos(theta1_1) * sin(theta4_2) + cos(theta4_2) * cos(theta2_1 + theta3_1) * sin(theta1_1)) * ay - cos(theta4_2) * sin(theta2_1 + theta3_1) * az, cos(theta1_1) * sin(theta2_1 + theta3_1) * ax + sin(theta1_1) * sin(theta2_1 + theta3_1) * ay + cos(theta2_1 + theta3_1) * az)
    theta5_3 = atan2((cos(theta1_1) * cos(theta4_3) * cos(theta2_2 + theta3_2) - sin(theta1_1) * sin(theta4_3)) * ax + (cos(theta1_1) * sin(theta4_3) + cos(theta4_3) * cos(theta2_2 + theta3_2) * sin(theta1_1)) * ay - cos(theta4_3) * sin(theta2_2 + theta3_2) * az, cos(theta1_1) * sin(theta2_2 + theta3_2) * ax + sin(theta1_1) * sin(theta2_2 + theta3_2) * ay + cos(theta2_2 + theta3_2) * az)
    theta5_4 = atan2((cos(theta1_1) * cos(theta4_4) * cos(theta2_2 + theta3_2) - sin(theta1_1) * sin(theta4_4)) * ax + (cos(theta1_1) * sin(theta4_4) + cos(theta4_4) * cos(theta2_2 + theta3_2) * sin(theta1_1)) * ay - cos(theta4_4) * sin(theta2_2 + theta3_2) * az, cos(theta1_1) * sin(theta2_2 + theta3_2) * ax + sin(theta1_1) * sin(theta2_2 + theta3_2) * ay + cos(theta2_2 + theta3_2) * az)
    theta5_5 = atan2((cos(theta1_2) * cos(theta4_5) * cos(theta2_3 + theta3_1) - sin(theta1_2) * sin(theta4_5)) * ax + (cos(theta1_2) * sin(theta4_5) + cos(theta4_5) * cos(theta2_3 + theta3_1) * sin(theta1_2)) * ay - cos(theta4_5) * sin(theta2_3 + theta3_1) * az, cos(theta1_2) * sin(theta2_3 + theta3_1) * ax + sin(theta1_2) * sin(theta2_3 + theta3_1) * ay + cos(theta2_3 + theta3_1) * az)
    theta5_6 = atan2((cos(theta1_2) * cos(theta4_6) * cos(theta2_3 + theta3_1) - sin(theta1_2) * sin(theta4_6)) * ax + (cos(theta1_2) * sin(theta4_6) + cos(theta4_6) * cos(theta2_3 + theta3_1) * sin(theta1_2)) * ay - cos(theta4_6) * sin(theta2_3 + theta3_1) * az, cos(theta1_2) * sin(theta2_3 + theta3_1) * ax + sin(theta1_2) * sin(theta2_3 + theta3_1) * ay + cos(theta2_3 + theta3_1) * az)
    theta5_7 = atan2((cos(theta1_2) * cos(theta4_7) * cos(theta2_4 + theta3_2) - sin(theta1_2) * sin(theta4_7)) * ax + (cos(theta1_2) * sin(theta4_7) + cos(theta4_7) * cos(theta2_4 + theta3_2) * sin(theta1_2)) * ay - cos(theta4_7) * sin(theta2_4 + theta3_2) * az, cos(theta1_2) * sin(theta2_4 + theta3_2) * ax + sin(theta1_2) * sin(theta2_4 + theta3_2) * ay + cos(theta2_4 + theta3_2) * az)
    theta5_8 = atan2((cos(theta1_2) * cos(theta4_8) * cos(theta2_4 + theta3_2) - sin(theta1_2) * sin(theta4_8)) * ax + (cos(theta1_2) * sin(theta4_8) + cos(theta4_8) * cos(theta2_4 + theta3_2) * sin(theta1_2)) * ay - cos(theta4_8) * sin(theta2_4 + theta3_2) * az, cos(theta1_2) * sin(theta2_4 + theta3_2) * ax + sin(theta1_2) * sin(theta2_4 + theta3_2) * ay + cos(theta2_4 + theta3_2) * az)

    output =[]
    sol1 = [theta1_1 / pi * 180,theta2_2 / pi * 180,theta3_2 / pi * 180,theta4_3/ pi * 180 ,theta5_3 / pi * 180,theta6_4/ pi * 180]
    if(RangeTest(sol1)==True):
        output.append(sol1)
    sol2 = [theta1_2 / pi * 180,theta2_4 / pi * 180,theta3_2 / pi * 180,theta4_7 / pi * 180,theta5_7 / pi * 180,theta6_8/ pi * 180]
    if(RangeTest(sol2)==True):
        #return sol2
        output.append(sol2)
    sol3 = [theta1_1/ pi * 180 ,theta2_1/ pi * 180, theta3_1/ pi * 180 ,theta4_1/ pi * 180 ,theta5_1/ pi * 180 ,theta6_2/ pi * 180]
    if(RangeTest(sol3)==True):
        output.append(sol3)
        #return sol3
    sol4 = [theta1_2 / pi * 180,theta2_3 / pi * 180,theta3_1 / pi * 180,theta4_5 / pi * 180,theta5_5 / pi * 180,theta6_6/ pi * 180]
    if(RangeTest(sol4)==True):
        output.append(sol4)
        #return sol4
    sol5 = [theta1_1 / pi * 180,theta2_2 / pi * 180,theta3_2 / pi * 180,theta4_4 / pi * 180,theta5_4 / pi * 180,theta6_3/ pi * 180]
    if(RangeTest(sol5)==True):
        output.append(sol5)
        #return sol5
    sol6 = [theta1_2 / pi * 180,theta2_4 / pi * 180,theta3_2 / pi * 180,theta4_8 / pi * 180,theta5_8 / pi * 180,theta6_7/ pi * 180]
    if(RangeTest(sol6)==True):
        output.append(sol6)
        #return sol6
    sol7 = [theta1_1 / pi * 180,theta2_1 / pi * 180,theta3_1 / pi * 180,theta4_2 / pi * 180,theta5_2 / pi * 180,theta6_1/ pi * 180]
    if(RangeTest(sol7)==True):
        output.append(sol7)
        #return sol7
    sol8 = [theta1_2 / pi * 180,theta2_3 / pi * 180,theta3_1 / pi * 180,theta4_6 / pi * 180,theta5_6 / pi * 180,theta6_5/ pi * 180]
    if(RangeTest(sol8)==True):
        output.append(sol8)
        #return sol8
    return output
# Out of range test & output the result
def RangeTest(sol):
    temp = 1
    if (sol[0] > 160 or sol[0] < -160):
        #print("θ 1 is out of range!\n")
        temp=0
    if (sol[1] > 125 or sol[1] < -125):
        #print("θ 2 is out of range!\n")
        temp = 0
    if (sol[2] > 135 or sol[2] < -135):
        #print("θ 3 is out of range!\n")
        temp = 0
    if (sol[3] > 140 or sol[3] < -140):
        #print("θ 4 is out of range!\n")
        temp = 0
    if (sol[4] > 100 or sol[4] < -100):
        #print("θ 5 is out of range!\n")
        temp = 0
    if (sol[5] > 260 or sol[5] < -260):
        #print("θ 6 is out of range!\n")
        temp = 0
    # test not a number
    if (isnan(sol[0]) or isnan(sol[1]) or isnan(sol[2]) or isnan(sol[3]) or isnan(sol[4]) or isnan(sol[5])):
        #print('There is no solution!')
        temp = 0
    #print('sol=', sol)
    #print("----------------------------------------------------------------\n")
    if(temp==1):
        return True
    else :
        return False

class Arrow3D(FancyArrowPatch):

    def __init__(self, xs, ys, zs, *args, **kwargs):
     FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
     self._verts3d = xs, ys, zs

    def draw(self, renderer):
     xs3d, ys3d, zs3d = self._verts3d
     xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
     self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
     FancyArrowPatch.draw(self, renderer)


# straight line portion
def straight_line(thetaA, thetaB, t):
    T = 0.5
    h = t/T
    q_x = (thetaB-thetaA)*h + thetaA
    q_v = (thetaB-thetaA)/T
    q_a = 0
    return q_x, q_v, q_a

# transition portion
def transition(thetaA, thetaB, thetaC, t):
    tacc = 0.2
    T = 0.5
    h = (t+tacc)/(2*tacc)
    deltaC = thetaC - thetaB
    deltaB = 0.4*thetaA - 0.4*thetaB
    q_x = (((deltaC*tacc/T) + deltaB)*(2-h)*h*h - 2*deltaB)*h + thetaB + deltaB
    q_v = (((deltaC*tacc/T) + deltaB)*(1.5-h)*2*h*h - deltaB)/tacc
    q_a = ((deltaC*tacc/T) + deltaB)*(1-h)*3*h/tacc/tacc
    return q_x, q_v, q_a

modeChoose = 1

#point a
A=np.array([[0,1,0,20],[-1,0,0,30],[0,0,1,20],[0,0,0,1]])
#B
B=np.array([[0,0,-1,-10],[-1,0,0,15],[0,1,0,30],[0,0,0,1]])
#C
C=np.array([[1,0,0,-25],[0,-1,0,10],[0,0,-1,-20],[0,0,0,1]])
#print(A)
#print(B)
#print(C)
Ja=task2(A)
Jb=task2(B)
Jc=task2(C)
print(Ja[0])
print(Jb[0])
print(Jc[0])
choose_angle1 = Ja[0]
choose_angle2 = Jb[2]
choose_angle3 = Jc[0]
#A16, x_y_z, no_use_1 = task1(joint_theta1_x[i], joint_theta2_x[i], joint_theta3_x[i], joint_theta4_x[i],joint_theta5_x[i],joint_theta6_x[i])

'''

# the angle and position we choose 
angle1, out1 = IK(nx1, ny1, nz1, ox1, oy1, oz1, ax1, ay1, az1, px1, py1, pz1)
choose_angle1 = angle1[1]
print("Joint move angle1 使用第二組解", choose_angle1)

angle2, out2 = IK(nx2, ny2, nz2, ox2, oy2, oz2, ax2, ay2, az2, px2, py2, pz2)
choose_angle2 = angle2[1]
print("Joint move angle1 使用第二組解", choose_angle2)

angle3, out3 = IK(nx3, ny3, nz3, ox3, oy3, oz3, ax3, ay3, az3, px3, py3, pz3)
choose_angle3 = angle3[3]
print("Joint move angle1 使用第四組解", choose_angle3)
'''
q_x_sum = []
q_v_sum = []
q_a_sum = []
t_x = []
t = 0
# t_x.append(t)
for k in range(501):  # t = 0~1sec ，0.002sec per point 

    if 0 <= t < 0.3:  # linear movement
        for i in range(6):  # six theta to calculate
            q_x, q_v, q_a = straight_line(choose_angle1[i], choose_angle2[i],
                                          t)  # find angle, Angular velocity, Angular acceleration
            q_x_sum.append(q_x)  # angle
            q_v_sum.append(q_v)  # Angular velocity
            q_a_sum.append(q_a)  # Angular acceleration

    elif 0.3 <= t < 0.7:  # transfer movement
        for i in range(6):
            q_x, q_v, q_a = transition(choose_angle1[i], choose_angle2[i], choose_angle3[i], t - 0.5)
            q_x_sum.append(q_x)
            q_v_sum.append(q_v)
            q_a_sum.append(q_a)

    elif 0.7 <= t:  # linear movement
        c = t - 0.5
        for i in range(6):
            q_x, q_v, q_a = straight_line(choose_angle2[i], choose_angle3[i], c)
            q_x_sum.append(q_x)
            q_v_sum.append(q_v)
            q_a_sum.append(q_a)
    t_x.append(t)
    t += 0.002

# prepare memory to store theta1 to theta6 angle, Angular velocity, Angular acceleration
joint_theta1_x = []
joint_theta1_v = []
joint_theta1_a = []

joint_theta2_x = []
joint_theta2_v = []
joint_theta2_a = []

joint_theta3_x = []
joint_theta3_v = []
joint_theta3_a = []

joint_theta4_x = []
joint_theta4_v = []
joint_theta4_a = []

joint_theta5_x = []
joint_theta5_v = []
joint_theta5_a = []

joint_theta6_x = []
joint_theta6_v = []
joint_theta6_a = []

# save theta1 to theta6 angle, Angular velocity, Angular acceleration
for i in range(3006):
    if i % 6 == 0:
        joint_theta1_x.append(q_x_sum[i])
        joint_theta1_v.append(q_v_sum[i])
        joint_theta1_a.append(q_a_sum[i])
    if i % 6 == 1:
        joint_theta2_x.append(q_x_sum[i])
        joint_theta2_v.append(q_v_sum[i])
        joint_theta2_a.append(q_a_sum[i])
    if i % 6 == 2:
        joint_theta3_x.append(q_x_sum[i])
        joint_theta3_v.append(q_v_sum[i])
        joint_theta3_a.append(q_a_sum[i])
    if i % 6 == 3:
        joint_theta4_x.append(q_x_sum[i])
        joint_theta4_v.append(q_v_sum[i])
        joint_theta4_a.append(q_a_sum[i])
    if i % 6 == 4:
        joint_theta5_x.append(q_x_sum[i])
        joint_theta5_v.append(q_v_sum[i])
        joint_theta5_a.append(q_a_sum[i])
    if i % 6 == 5:
        joint_theta6_x.append(q_x_sum[i])
        joint_theta6_v.append(q_v_sum[i])
        joint_theta6_a.append(q_a_sum[i])

# print theta1 to theta6 angle
# joint 1
plt.figure()
plt.subplot(3, 2, 1)
plt.scatter(t_x, joint_theta1_x,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-65, 8)
plt.ylabel("Angle(°)")
plt.title("joint 1")

# joint 2
plt.subplot(3, 2, 2)
plt.scatter(t_x, joint_theta2_x,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-20, 123)
plt.ylabel("Angle")
plt.title("joint 2")

# joint 3
plt.subplot(3, 2, 3)
plt.scatter(t_x, joint_theta3_x,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(15, 26.3)
plt.ylabel("Length")
plt.title("joint 3")

# joint 4
plt.subplot(3, 2, 4)
plt.scatter(t_x, joint_theta4_x,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(0, 100)
plt.ylabel("Angle")
plt.title("joint 4")

# joint 5
plt.subplot(3, 2, 5)
plt.scatter(t_x, joint_theta5_x,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-88, -52)
plt.xlabel("time")
plt.ylabel("Angle")
plt.title("joint 5")

# joint 6
plt.subplot(3, 2, 6)
plt.scatter(t_x, joint_theta6_x,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-120, -55)
plt.xlabel("time")
plt.ylabel("Angle")
plt.title("joint 6")
# plt.show()
# plt.savefig("score.png")

# print theta1 to theta6 Angular velocity
# joint 1
plt.figure()
plt.subplot(3, 2, 1)
plt.scatter(t_x, joint_theta1_v,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-100, 200)
plt.ylabel("Angular Velocity(°/s)")
plt.title("joint 1")

# joint 2
plt.subplot(3, 2, 2)
plt.scatter(t_x, joint_theta2_v,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-142.5, -138.5)
plt.ylabel("Angular Velocity(°/s)")
plt.title("joint 2")

# joint 3
plt.subplot(3, 2, 3)
plt.scatter(t_x, joint_theta3_v,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-5, 22)
plt.ylabel("Velocity(cm/s)")
plt.title("joint 3")

# joint 4
plt.subplot(3, 2, 4)
plt.scatter(t_x, joint_theta4_v,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-200, 200)
plt.ylabel("Angular Velocity(°/s)")
plt.title("joint 4")

# joint 5
plt.subplot(3, 2, 5)
plt.scatter(t_x, joint_theta5_v,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-100, 50)
plt.xlabel("time")
plt.ylabel("Angular Velocity(°/s)")
plt.title("joint 5")

# joint 6
plt.subplot(3, 2, 6)
plt.scatter(t_x, joint_theta6_v,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-120, 20)
plt.xlabel("time")
plt.ylabel("Angular Velocity(°/s)")
plt.title("joint 6")
# plt.show()

# print theta1 to theta6 Angular acceleration
# joint 1
plt.figure()
plt.subplot(3, 2, 1)
plt.scatter(t_x, joint_theta1_a,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(0, 1000)
plt.ylabel("Angular Acceleration(°/s²)")
plt.title("joint 1")

# joint 2
plt.subplot(3, 2, 2)
plt.scatter(t_x, joint_theta2_a,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(0, 12)
plt.ylabel("Angular Acceleration(°/s²)")
plt.title("joint 2")

# joint 3
plt.subplot(3, 2, 3)
plt.scatter(t_x, joint_theta3_a,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(0, 100)
plt.ylabel("Acceleration(cm/s²)")
plt.title("joint 3")

# joint 4
plt.subplot(3, 2, 4)
plt.scatter(t_x, joint_theta4_a,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(0, 1500)
plt.ylabel("Angular Acceleration(°/s²)")
plt.title("joint 4")

# joint 5
plt.subplot(3, 2, 5)
plt.scatter(t_x, joint_theta5_a,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(-430, 0)
plt.xlabel("time")
plt.ylabel("Angular Acceleration(°/s²)")
plt.title("joint 5")

# joint 6
plt.subplot(3, 2, 6)
plt.scatter(t_x, joint_theta6_a,marker = '.')
##plt.xlim(0, 1)
##plt.ylim(0, 600)
plt.xlabel("time")
plt.ylabel("Angular Acceleration(°/s²)")
plt.title("joint 6")
plt.show()

# 3D path of Joint Motion
x = []
y = []
z = []
arrow_x = []
arrow_y = []
arrow_z = []
# get each position and noap's "a" vector every 0.02sec
for i in range(501):
    '''try:
        A16, x_y_z, no_use_1 = task1(joint_theta1_x[i], joint_theta2_x[i], joint_theta3_x[i], joint_theta4_x[i],
                                     joint_theta5_x[i], joint_theta6_x[i])
    except:
        print(joint_theta5_x[i],joint_theta6_x[i])'''
    #print(joint_theta1_x[i], joint_theta2_x[i], joint_theta3_x[i], joint_theta4_x[i],
                                 #joint_theta5_x[i],
                                # joint_theta6_x[i])
    A16, x_y_z, no_use_1 = task1(joint_theta1_x[i], joint_theta2_x[i], joint_theta3_x[i], joint_theta4_x[i],
                                 joint_theta5_x[i],
                                 joint_theta6_x[i])
    x.append(x_y_z[0])
    y.append(x_y_z[1])
    z.append(x_y_z[2])
    arrow_x.append(A16[0][2])
    arrow_y.append(A16[1][2])
    arrow_z.append(A16[2][2])

# print 3D picture
ax = plt.subplot(projection='3d')

x=np.multiply(x, 100)
y=np.multiply(y, 100)
z=np.multiply(z, 100)

ax.scatter(x, y, z, c='b')

text = str(round(x[0],3)) + ', ' + str(round(y[0],3)) + ', ' + str(round(z[0],3))
ax.text(x[0], y[0], z[0], text, fontsize=10)
text = str(round(x[250],3)) + ', ' + str(round(y[250],3)) + ', ' + str(round(z[250],3))
ax.text(x[250], y[250], z[250], text, fontsize=10)
text = str(round(x[500],3)) + ', ' + str(round(y[500],3)) + ', ' + str(round(z[500],3))
ax.text(x[500], y[500], z[500], text, fontsize=10)

ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')

# coordinate A in 3D
#print(x[0],y[0],z[0])
a = Arrow3D([x[0], x[0]], [y[0], y[0]], [z[0], z[0] + 10], mutation_scale=2, lw=1, arrowstyle="->", color="b")
b = Arrow3D([x[0], x[0] + 10], [y[0], y[0]], [z[0], z[0]], mutation_scale=2, lw=1, arrowstyle="->", color="g")
c = Arrow3D([x[0], x[0]], [y[0], y[0] - 5], [z[0], z[0]], mutation_scale=2, lw=1, arrowstyle="->", color="r")
ax.add_artist(a)
ax.add_artist(b)
ax.add_artist(c)

# coordinate B in 3D
a = Arrow3D([x[250], x[250] - 8], [y[250], y[250]], [z[250], z[250]], mutation_scale=2, lw=1, arrowstyle="->",
            color="b")
b = Arrow3D([x[250], x[250]], [y[250], y[250] - 10], [z[250], z[250]], mutation_scale=2, lw=1, arrowstyle="->",
            color="r")
c = Arrow3D([x[250], x[250]], [y[250], y[250]], [z[250], z[250] + 10], mutation_scale=2, lw=1, arrowstyle="->",
            color="g")
ax.add_artist(a)
ax.add_artist(b)
ax.add_artist(c)

# coordinate C in 3D
a = Arrow3D([x[500], x[500] ], [y[500], y[500]], [z[500], z[500]-10], mutation_scale=2, lw=1, arrowstyle="->",
            color="b")
b = Arrow3D([x[500], x[500]+10], [y[500], y[500]], [z[500], z[500] ], mutation_scale=2, lw=1, arrowstyle="->",
            color="r")
c = Arrow3D([x[500], x[500]], [y[500], y[500] - 5], [z[500], z[500]], mutation_scale=2, lw=1, arrowstyle="->",
            color="g")
ax.add_artist(a)
ax.add_artist(b)
ax.add_artist(c)

"""
# draw [noap]'s "a" vector every 0.03sec
for i in range(0, 499, 3):
    c = Arrow3D([x[i + 1], x[i + 1] + 0.01 * arrow_x[i + 1]], [y[i + 1], y[i + 1] + 0.01* arrow_y[i + 1]],
                [z[i + 1], z[i + 1] + 0.01 * arrow_z[i + 1]], mutation_scale=10, lw=2, arrowstyle="->", color="g")
    #ax.add_artist(c)
"""
plt.show()


#---------------------------------Cartesian Move
no_use_2, x_y_z1, p_t_s_1 = task1(choose_angle1[0], choose_angle1[1], choose_angle1[2], choose_angle1[3],
                                   choose_angle1[4], choose_angle1[5])
no_use_3, x_y_z2, p_t_s_2 = task1(choose_angle2[0], choose_angle2[1], choose_angle2[2], choose_angle2[3],
                                   choose_angle2[4], choose_angle2[5])
no_use_4, x_y_z3, p_t_s_3 = task1(choose_angle3[0], choose_angle3[1], choose_angle3[2], choose_angle3[3],
                                   choose_angle3[4], choose_angle3[5])

p_t_s1 = []
if abs(p_t_s_3[2] - p_t_s_1[2]) * 180 / pi > 90:  # if |saiC-saiA| > 90 degree, saiA + 180 degree, thetaA = -thetaA
    p_t_s1.append(p_t_s_1[0] * 180 / pi)
    p_t_s1.append(-p_t_s_1[1] * 180 / pi)
    p_t_s1.append(p_t_s_1[2] * 180 / pi + 180)
else:
    for i in range(3):
        p_t_s1.append(p_t_s_1[i] * 180 / pi)
p_t_s2 = []
p_t_s3 = []
for i in range(3):
    p_t_s2.append(p_t_s_2[i] * 180 / pi)
    p_t_s3.append(p_t_s_3[i] * 180 / pi)

cartesian_x_sum = []
cartesian_v_sum = []
cartesian_a_sum = []
cartesian_t_x = []
cartesian_t = 0
for k in range(501):  # cartesian_t = 0~1sec ，0.002sec per point

    if 0 <= cartesian_t < 0.3:  # linear movement
        for i in range(3):  # calculate x, y, z
            q_x, q_v, q_a = straight_line(x_y_z1[i], x_y_z2[i],
                                          cartesian_t)  # find angle, Angular velocity, Angular acceleration
            cartesian_x_sum.append(q_x)  # angle
            cartesian_v_sum.append(q_v)  # Angular velocity
            cartesian_a_sum.append(q_a)  # Angular acceleration

    elif 0.3 <= cartesian_t < 0.7:  # transfer movement
        for i in range(3):
            q_x, q_v, q_a = transition(x_y_z1[i], x_y_z2[i], x_y_z3[i], cartesian_t - 0.5)
            cartesian_x_sum.append(q_x)
            cartesian_v_sum.append(q_v)
            cartesian_a_sum.append(q_a)

    elif 0.7 <= cartesian_t:  # linear movement
        c = cartesian_t - 0.5
        for i in range(3):
            q_x, q_v, q_a = straight_line(x_y_z2[i], x_y_z3[i], c)
            cartesian_x_sum.append(q_x)
            cartesian_v_sum.append(q_v)
            cartesian_a_sum.append(q_a)
    cartesian_t_x.append(cartesian_t)
    cartesian_t += 0.002

# prepare memory to store x, y, z, their Angular velocity, Angular acceleration
cartesian_x_x = []
cartesian_x_v = []
cartesian_x_a = []

cartesian_y_x = []
cartesian_y_v = []
cartesian_y_a = []

cartesian_z_x = []
cartesian_z_v = []
cartesian_z_a = []

# save theta1 to theta6 angle, Angular velocity, Angular acceleration
for i in range(1503):
    if i % 3 == 0:
        cartesian_x_x.append(cartesian_x_sum[i]*100)
        cartesian_x_v.append(cartesian_v_sum[i]*100)
        cartesian_x_a.append(cartesian_a_sum[i]*100)

    if i % 3 == 1:
        cartesian_y_x.append(cartesian_x_sum[i]*100)
        cartesian_y_v.append(cartesian_v_sum[i]*100)
        cartesian_y_a.append(cartesian_a_sum[i]*100)

    if i % 3 == 2:
        cartesian_z_x.append(cartesian_x_sum[i]*100)
        cartesian_z_v.append(cartesian_v_sum[i]*100)
        cartesian_z_a.append(cartesian_a_sum[i]*100)

# cartesian_x
plt.figure()
plt.subplot(3, 1, 1)
plt.scatter(cartesian_t_x, cartesian_x_x, marker='.')
#plt.xlim(0, 1)
#plt.ylim(-10, 20.5)
plt.ylabel("Place(cm)")
plt.title("position of x")

# cartesian_y
plt.subplot(3, 1, 2)
plt.scatter(cartesian_t_x, cartesian_y_x, marker='.')
#plt.xlim(0, 1)
#plt.ylim(-5, 15)
plt.ylabel("Place(cm)")
plt.title("position of y")

# cartesian_z
plt.subplot(3, 1, 3)
plt.scatter(cartesian_t_x, cartesian_z_x, marker='.')
#plt.xlim(0, 1)
#plt.ylim(-10, 25)
plt.ylabel("Place(cm)")
plt.title("position of z")
plt.xlabel("time")

# print x, y, z Angular velocity
# cartesian_x
plt.figure()
plt.subplot(3, 1, 1)
plt.scatter(cartesian_t_x, cartesian_x_v, marker='.')
#plt.xlim(0, 1)
#plt.ylim(-60, 0)
plt.ylabel("Velocity(cm/s)")
plt.title("velocity of x")

# cartesian_y
plt.subplot(3, 1, 2)
plt.scatter(cartesian_t_x, cartesian_y_v, marker='.')
#plt.xlim(0, 1)
#plt.ylim(-30, 40)
plt.ylabel("Velocity(cm/s)")
plt.title("velocity of y")

# cartesian_z
plt.subplot(3, 1, 3)
plt.scatter(cartesian_t_x, cartesian_z_v, marker='.')
#plt.xlim(0, 1)
#plt.ylim(30, 40)
plt.ylabel("Velocity(cm/s)")
plt.title("velocity of z")
plt.xlabel("time")

# print x, y, z Angular velocity
# cartesian_x
plt.figure()
plt.subplot(3, 1, 1)
plt.scatter(cartesian_t_x, cartesian_x_a, marker='.')
#plt.xlim(0, 1)
#plt.ylim(-230, 0)
plt.ylabel("Acceleration(cm/s)")
plt.title("acceleration of x")

# cartesian_y
plt.subplot(3, 1, 2)
plt.scatter(cartesian_t_x, cartesian_y_a, marker='.')
#plt.xlim(0, 1)
#plt.ylim(0, 270)
plt.ylabel("Acceleration(cm/s)")
plt.title("acceleration of y")

# cartesian_z
plt.subplot(3, 1, 3)
plt.scatter(cartesian_t_x, cartesian_z_a, marker='.')
#plt.xlim(0, 1)
#plt.ylim(-40, 0)
plt.ylabel("Acceleration(cm/s)")
plt.title("acceleration of z")
plt.xlabel("time")
plt.show()

# 3D path of Joint Motion
x = []
y = []
z = []
c_arrow_x = []
c_arrow_y = []
c_arrow_z = []
# get each position and noap's "a" vector every 0.02sec
for i in range(501):
    x.append(cartesian_x_x[i])
    y.append(cartesian_y_x[i])
    z.append(cartesian_z_x[i])
    c_arrow_x.append(A16[0][2])
    c_arrow_y.append(A16[1][2])
    c_arrow_z.append(A16[2][2])

# print 3D picture
ax = plt.subplot(projection='3d')
ax.scatter(x, y, z, c='b')

text = str(round(x[0], 4)) + ', ' + str(round(y[0], 4)) + ', ' + str(round(z[0], 4))
ax.text(x[0], y[0], z[0], text, fontsize=10)
text = str(round(x[250], 4)) + ', ' + str(round(y[250], 4)) + ', ' + str(round(z[250], 4))
ax.text(x[250], y[250], z[250], text, fontsize=10)
text = str(round(x[500], 4)) + ', ' + str(round(y[500], 4)) + ', ' + str(round(z[500], 4))
ax.text(x[500], y[500], z[500], text, fontsize=10)

ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')

# coordinate A in 3D
a = Arrow3D([x[0], x[0]], [y[0], y[0]], [z[0], z[0] - 10], mutation_scale=20, lw=2, arrowstyle="->", color="b")
b = Arrow3D([x[0], x[0] + 10], [y[0], y[0]], [z[0], z[0]], mutation_scale=20, lw=2, arrowstyle="->", color="r")
c = Arrow3D([x[0], x[0]], [y[0], y[0] - 10], [z[0], z[0]], mutation_scale=20, lw=2, arrowstyle="->", color="g")
ax.add_artist(a)
ax.add_artist(b)
ax.add_artist(c)

# coordinate B in 3D
a = Arrow3D([x[250], x[250] - 10], [y[250], y[250]], [z[250], z[250]], mutation_scale=20, lw=2, arrowstyle="->",
            color="b")
b = Arrow3D([x[250], x[250]], [y[250], y[250] - 5], [z[250], z[250]], mutation_scale=20, lw=2, arrowstyle="->",
            color="r")
c = Arrow3D([x[250], x[250]], [y[250], y[250]], [z[250], z[250] + 10], mutation_scale=20, lw=2, arrowstyle="->",
            color="g")
ax.add_artist(a)
ax.add_artist(b)
ax.add_artist(c)

# coordinate C in 3D
a = Arrow3D([x[500], x[500] + 10], [y[500], y[500]], [z[500], z[500]], mutation_scale=20, lw=2, arrowstyle="->",
            color="b")
b = Arrow3D([x[500], x[500]], [y[500], y[500]], [z[500], z[500] + 10], mutation_scale=20, lw=2, arrowstyle="->",
            color="r")
c = Arrow3D([x[500], x[500]], [y[500], y[500] - 5], [z[500], z[500]], mutation_scale=20, lw=2, arrowstyle="->",
            color="g")
ax.add_artist(a)
ax.add_artist(b)
ax.add_artist(c)

plt.show()

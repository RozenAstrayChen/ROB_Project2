from math import *
import numpy as np
import matplotlib.pyplot as plt # show the picure
from mpl_toolkits.mplot3d import Axes3D # show 3D picture
from matplotlib.patches import FancyArrowPatch # draw a vector
from mpl_toolkits.mplot3d import proj3d # draw 3D vector
import Table
import utlis
np.set_printoptions(suppress=True)

T=np.zeros((4,4))
A = np.zeros((4, 4, 6))

def forward_K(t1, t2, t3, t4, t5, t6):
    puma560 = Table.getPUMA560(input=[t1, t2, t3, t4, t5, t6])

    res = np.eye(4, dtype='float32')
    for joint in puma560:
        a = Table.getTable(joint[0], joint[1], joint[2], joint[3])
        a = ((a*10000).astype('int32')/10000).astype('float32')
        res = np.matmul(res, a)

    x = res[0][3]; y = res[1][3];z = res[2][3];

    phi = atan2(res[1][2], res[0][2])/pi*180
    theata = atan2(((res[0][2]**2+res[1][2]**2)**0.5),res[2][2])/ pi*180
    psi = atan2(res[2][1]/sin(theata),-res[2][0]/sin(theata)) / pi*180
    #print('NOAP Table is :\n')
    #print(res)
    #print("x, y, z = %f, %f, %f" %(x, y, z))
    #print("phi, theata, psi = %f, %f, %f" % (phi, theata, psi))

    xyz = [x, y, z]
    pos = [phi, theata, psi]
    A = T

    return A, xyz, pos

def inverse_K(Tn):
    '''
    inverse kinematics
    '''
    ik = Table.inverseKinematics(Tn)
    t1 = ik.findTheta_1()
    t3 = ik.findTheta_3()
    t2 = ik.findTheta_2()
    t4 = ik.findTheta_4()
    t6 = ik.findTheta_6()
    t5 = ik.findTheta_5()
    '''[50.00111534 50.04030689 49.93981155 49.98904582 50.01993304 50.0247087 ]'''
    sol1 = np.array([t1[0], t2[0], t3[0], t4[0], t5[0], t6[1]] ) / pi * 180
    '''[ 50.00111534   7.25988379 135.34934185 -82.94144809 -36.25522717 168.72359923]'''
    sol2 = np.array([t1[0], t2[1], t3[1], t4[3], t5[3], t6[2]] ) / pi * 180
    '''[  50.00111534   50.04030689   49.93981155 -130.01095418  -50.01993304 -129.9752913 ]'''
    sol3 = np.array([t1[0], t2[0], t3[0], t4[1], t5[1], t6[0]] ) / pi * 180
    '''[ 50.00111534   7.25988379 135.34934185  97.05855191  36.25522717 -11.27640077]'''
    sol4 = np.array([t1[0], t2[1], t3[1], t4[2], t5[2], t6[3]] ) / pi * 180
    sol5 = np.array([t1[1], t2[2], t3[0], t4[4], t5[4], t6[5]] ) / pi * 180
    '''[-106.20900635 -230.04030689  135.34934185 -149.37865474   38.37480716 80.82747849]'''
    sol6 = np.array([t1[1], t2[3], t3[1], t4[6], t5[6], t6[7]]) / pi * 180
    '''[-106.20900635 -187.25988379   49.93981155  113.55842631   -20.17968307 170.79313242]'''
    sol7 = np.array([t1[1], t2[2], t3[0], t4[5], t5[5], t6[4]]) / pi * 180
    '''[-106.20900635 -230.04030689  135.34934185   30.62134526  -38.37480716 -99.17252151]'''
    sol8 = np.array([t1[1], t2[3], t3[1], t4[7], t5[7], t6[6]]) / pi * 180

    soln = [sol1, sol2, sol3, sol4, sol5, sol6, sol7, sol8]
    return  utlis.judgeangle(soln)
    

if __name__ == "__main__":
    modeChoose = 1

    #point a
    A=np.array(
        [[0, 1, 0, 20], [-1, 0, 0, 30], [0, 0, 1, 20], [0, 0, 0, 1]]
        )
    #B
    B=np.array(
        [[0,0,-1,-10],[-1,0,0,15],[0,1,0,30],[0,0,0,1]]
        )
    #C
    C=np.array([[1,0,0,-25],[0,-1,0,10],[0,0,-1,-20],[0,0,0,1]])

    Ja=inverse_K(A); Jb=inverse_K(B); Jc=inverse_K(C)

    choose_angle1 = Ja[0]; choose_angle2 = Jb[2]; choose_angle3 = Jc[0]
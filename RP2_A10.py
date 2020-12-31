from math import *
import numpy as np
import matplotlib.pyplot as plt  # show the picure
from mpl_toolkits.mplot3d import Axes3D  # show 3D picture
from matplotlib.patches import FancyArrowPatch  # draw a vector
from mpl_toolkits.mplot3d import proj3d  # draw 3D vector
import Table
import utlis
np.set_printoptions(suppress=True)

T = np.zeros((4, 4))
A = np.zeros((4, 4, 6))


def forward_K(t1, t2, t3, t4, t5, t6):
    '''puma560 = Table.getPUMA560(input=[t1, t2, t3, t4, t5, t6])

    res = np.eye(4, dtype='float32')
    for joint in puma560:
        a = Table.getTable(joint[0], joint[1], joint[2], joint[3])
        #a = ((a*10000).astype('int32')/10000).astype('float32')
        res = np.matmul(res, a)

    x = res[0][3]; y = res[1][3];z = res[2][3];

    phi = atan2(res[1][2], res[0][2])/pi*180
    theata = atan2(((res[0][2]**2+ res[1][2]**2)**0.5), res[2][2])/ pi*180
    psi = atan2(res[2][1]/sin(theata),-res[2][0]/sin(theata)) / pi*180
    #print('NOAP Table is :\n')
    #print(res)
    #print("x, y, z = %f, %f, %f" %(x, y, z))
    #print("phi, theata, psi = %f, %f, %f" % (phi, theata, psi))

    xyz = [x, y, z]
    pos = [phi, theata, psi]
    A = T

    return A, xyz, pos'''
    ap = np.array([(-90 / 180 * pi), 0, (90 / 180 * pi),
                   (-90 / 180 * pi), (90 / 180 * pi), 0])
    a = np.array([0, 0.432, -0.02, 0, 0, 0])
    d = np.array([0, 0, 0.149, 0.433, 0, 0])
    theta = [
        t1 / 180 * pi,
        t2 / 180 * pi,
        t3 / 180 * pi,
        t4 / 180 * pi,
        t5 / 180 * pi,
        t6 / 180 * pi]
    T = np.zeros((4, 4))
    A = np.zeros((4, 4, 6))
    # calculate A1~A6
    for i in range(6):
        A[:, :, i] = [[cos(theta[i]), -
                       sin(theta[i]) *
                       cos(ap[i]), sin(theta[i]) *
                       sin(ap[i]), a[i] *
                       cos(theta[i])], [sin(theta[i]), cos(theta[i]) *
                                        cos(ap[i]), -
                                        cos(theta[i]) *
                                        sin(ap[i]), a[i] *
                                        sin(theta[i])], [0, sin(ap[i]), cos(ap[i]), d[i]], [0, 0, 0, 1]]
    # calculate T=A1*A2*A3*A4*A5*A6
    T = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(
        A[:, :, 0], A[:, :, 1]), A[:, :, 2]), A[:, :, 3]), A[:, :, 4]), A[:, :, 5])
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    phi = atan2(T[1, 2], T[0, 2]) / pi * 180
    theta = atan2(((T[0, 2]**2 + T[1, 2]**2)**0.5), T[2, 2]) / pi * 180
    psi = atan2(T[2, 1], -T[2, 0]) / pi * 180

    xyz = [x, y, z]
    position = [phi, theta, psi]
    A = T
    return A, xyz, position


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
    sol1 = np.array([t1[0], t2[0], t3[0], t4[0], t5[0], t6[1]]) / pi * 180
    '''[ 50.00111534   7.25988379 135.34934185 -82.94144809 -36.25522717 168.72359923]'''
    sol2 = np.array([t1[0], t2[1], t3[1], t4[3], t5[3], t6[2]]) / pi * 180
    '''[  50.00111534   50.04030689   49.93981155 -130.01095418  -50.01993304 -129.9752913 ]'''
    sol3 = np.array([t1[0], t2[0], t3[0], t4[1], t5[1], t6[0]]) / pi * 180
    '''[ 50.00111534   7.25988379 135.34934185  97.05855191  36.25522717 -11.27640077]'''
    sol4 = np.array([t1[0], t2[1], t3[1], t4[2], t5[2], t6[3]]) / pi * 180
    sol5 = np.array([t1[1], t2[2], t3[0], t4[4], t5[4], t6[5]]) / pi * 180
    '''[-106.20900635 -230.04030689  135.34934185 -149.37865474   38.37480716 80.82747849]'''
    sol6 = np.array([t1[1], t2[3], t3[1], t4[6], t5[6], t6[7]]) / pi * 180
    '''[-106.20900635 -187.25988379   49.93981155  113.55842631   -20.17968307 170.79313242]'''
    sol7 = np.array([t1[1], t2[2], t3[0], t4[5], t5[5], t6[4]]) / pi * 180
    '''[-106.20900635 -230.04030689  135.34934185   30.62134526  -38.37480716 -99.17252151]'''
    sol8 = np.array([t1[1], t2[3], t3[1], t4[7], t5[7], t6[6]]) / pi * 180

    soln = [sol1, sol2, sol3, sol4, sol5, sol6, sol7, sol8]
    return utlis.judgeangle(soln)


if __name__ == "__main__":
    modeChoose = 1

    # point a
    A = np.array(
        [[0, 1, 0, 20], [-1, 0, 0, 30], [0, 0, 1, 20], [0, 0, 0, 1]]
    )
    # B
    B = np.array(
        [[0, 0, -1, -10], [-1, 0, 0, 15], [0, 1, 0, 30], [0, 0, 0, 1]]
    )
    # C
    C = np.array([[1, 0, 0, -25], [0, -1, 0, 10],
                  [0, 0, -1, -20], [0, 0, 0, 1]])

    Ja = inverse_K(A)
    Jb = inverse_K(B)
    Jc = inverse_K(C)
    print(Ja)
    choose_angle1 = Ja[0]
    choose_angle2 = Jb[2]
    choose_angle3 = Jc[0]

    q_x_sum = []
    q_v_sum = []
    q_a_sum = []
    t_x = []
    t = 0
    for k in range(501):  # t = 0~1sec ，0.002sec per point

        if 0 <= t < 0.3:  # linear movement
            for i in range(6):  # six theta to calculate
                # find angle, Angular velocity, Angular acceleration
                q_x, q_v, q_a = utlis.straight_line(
                    choose_angle1[i], choose_angle2[i], t)
                q_x_sum.append(q_x)  # angle
                q_v_sum.append(q_v)  # Angular velocity
                q_a_sum.append(q_a)  # Angular acceleration

        elif 0.3 <= t < 0.7:  # transfer movement
            for i in range(6):
                q_x, q_v, q_a = utlis.transition(
                    choose_angle1[i], choose_angle2[i], choose_angle3[i], t - 0.5)
                q_x_sum.append(q_x)
                q_v_sum.append(q_v)
                q_a_sum.append(q_a)

        elif 0.7 <= t:  # linear movement
            c = t - 0.5
            for i in range(6):
                q_x, q_v, q_a = utlis.straight_line(
                    choose_angle2[i], choose_angle3[i], c)
                q_x_sum.append(q_x)
                q_v_sum.append(q_v)
                q_a_sum.append(q_a)

        t_x.append(t)
        t += 0.002

    # prepare memory to store theta1 to theta6 angle, Angular velocity,
    # Angular acceleration
    joint_theta1_p = []
    joint_theta1_v = []
    joint_theta1_a = []
    joint_theta2_p = []
    joint_theta2_v = []
    joint_theta2_a = []
    joint_theta3_p = []
    joint_theta3_v = []
    joint_theta3_a = []
    joint_theta4_p = []
    joint_theta4_v = []
    joint_theta4_a = []
    joint_theta5_p = []
    joint_theta5_v = []
    joint_theta5_a = []
    joint_theta6_p = []
    joint_theta6_v = []
    joint_theta6_a = []

    # save theta1 to theta6 angle, Angular velocity, Angular acceleration
    for i in range(3006):
        if i % 6 == 0:
            joint_theta1_p.append(q_x_sum[i])
            joint_theta1_v.append(q_v_sum[i])
            joint_theta1_a.append(q_a_sum[i])
        if i % 6 == 1:
            joint_theta2_p.append(q_x_sum[i])
            joint_theta2_v.append(q_v_sum[i])
            joint_theta2_a.append(q_a_sum[i])
        if i % 6 == 2:
            joint_theta3_p.append(q_x_sum[i])
            joint_theta3_v.append(q_v_sum[i])
            joint_theta3_a.append(q_a_sum[i])
        if i % 6 == 3:
            joint_theta4_p.append(q_x_sum[i])
            joint_theta4_v.append(q_v_sum[i])
            joint_theta4_a.append(q_a_sum[i])
        if i % 6 == 4:
            joint_theta5_p.append(q_x_sum[i])
            joint_theta5_v.append(q_v_sum[i])
            joint_theta5_a.append(q_a_sum[i])
        if i % 6 == 5:
            joint_theta6_p.append(q_x_sum[i])
            joint_theta6_v.append(q_v_sum[i])
            joint_theta6_a.append(q_a_sum[i])

    # theta1 to theta6 angle
    utlis.plot(t_x,
               [joint_theta1_p,
                joint_theta2_p,
                joint_theta3_p,
                joint_theta4_p,
                joint_theta5_p,
                joint_theta6_p])
    # theta1 to theta6 Angular velocity
    utlis.plot(t_x,
               [joint_theta1_v,
                joint_theta2_v,
                joint_theta3_v,
                joint_theta4_v,
                joint_theta5_v,
                joint_theta6_v],
               y_label='Angular Velocity(°/s)')
    # theta1 to theta6 Angular acceleration
    utlis.plot(t_x,
               [joint_theta1_a,
                joint_theta2_a,
                joint_theta3_a,
                joint_theta4_a,
                joint_theta4_a,
                joint_theta4_a],
               y_label='Angular Acceleration(°/s²)')
    plt.show()
    # 3D path of Joint Motion
    x = []
    y = []
    z = []
    arrow_x = []
    arrow_y = []
    arrow_z = []
    for i in range(501):

        A16, x_y_z, _ = forward_K(joint_theta1_p[i],
                                  joint_theta2_p[i],
                                  joint_theta3_p[i],
                                  joint_theta4_p[i],
                                  joint_theta5_p[i],
                                  joint_theta6_p[i])
        x.append(x_y_z[0])
        y.append(x_y_z[1])
        z.append(x_y_z[2])
        arrow_x.append(A16[0][2])
        arrow_y.append(A16[1][2])
        arrow_z.append(A16[2][2])

    # print 3D picture
    ax = plt.subplot(projection='3d')

    x = np.multiply(x, 100)
    y = np.multiply(y, 100)
    z = np.multiply(z, 100)

    ax.scatter(x, y, z, c='b')

    text = str(round(x[0], 3)) + ', ' + \
        str(round(y[0], 3)) + ', ' + str(round(z[0], 3))
    ax.text(x[0], y[0], z[0], text, fontsize=10)
    text = str(round(x[250], 3)) + ', ' + \
        str(round(y[250], 3)) + ', ' + str(round(z[250], 3))
    ax.text(x[250], y[250], z[250], text, fontsize=10)
    text = str(round(x[500], 3)) + ', ' + \
        str(round(y[500], 3)) + ', ' + str(round(z[500], 3))
    ax.text(x[500], y[500], z[500], text, fontsize=10)

    ax.set_zlabel('Z')
    ax.set_ylabel('Y')
    ax.set_xlabel('X')

    # coordinate A in 3D
    # print(x[0],y[0],z[0])
    a = utlis.Arrow3D([x[0], x[0]], [y[0], y[0]], [z[0], z[0] +
                                                   10], mutation_scale=2, lw=1, arrowstyle="->", color="b")
    b = utlis.Arrow3D([x[0], x[0] + 10], [y[0], y[0]], [z[0], z[0]],
                      mutation_scale=2, lw=1, arrowstyle="->", color="g")
    c = utlis.Arrow3D([x[0], x[0]], [y[0], y[0] - 5], [z[0], z[0]],
                      mutation_scale=2, lw=1, arrowstyle="->", color="r")
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)

    # coordinate B in 3D
    a = utlis.Arrow3D([x[250], x[250] - 8], [y[250], y[250]], [z[250],
                                                               z[250]], mutation_scale=2, lw=1, arrowstyle="->", color="b")
    b = utlis.Arrow3D([x[250], x[250]], [y[250], y[250] - 10], [z[250],
                                                                z[250]], mutation_scale=2, lw=1, arrowstyle="->", color="r")
    c = utlis.Arrow3D([x[250], x[250]], [y[250], y[250]], [
                      z[250], z[250] + 10], mutation_scale=2, lw=1, arrowstyle="->", color="g")
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)

    # coordinate C in 3D
    a = utlis.Arrow3D([x[500], x[500]], [y[500], y[500]], [
                      z[500], z[500] - 10], mutation_scale=2, lw=1, arrowstyle="->", color="b")
    b = utlis.Arrow3D([x[500], x[500] + 10], [y[500], y[500]], [z[500],
                                                                z[500]], mutation_scale=2, lw=1, arrowstyle="->", color="r")
    c = utlis.Arrow3D([x[500], x[500]], [y[500], y[500] - 5], [z[500],
                                                               z[500]], mutation_scale=2, lw=1, arrowstyle="->", color="g")
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)
    plt.show()

    # ---------------------------------Cartesian Move
    _, x_y_z1, p_t_s_1 = forward_K(choose_angle1[0], choose_angle1[1], choose_angle1[2], choose_angle1[3],
                                   choose_angle1[4], choose_angle1[5])
    _, x_y_z2, p_t_s_2 = forward_K(choose_angle2[0], choose_angle2[1], choose_angle2[2], choose_angle2[3],
                                   choose_angle2[4], choose_angle2[5])
    _, x_y_z3, p_t_s_3 = forward_K(choose_angle3[0], choose_angle3[1], choose_angle3[2], choose_angle3[3],
                                   choose_angle3[4], choose_angle3[5])

    p_t_s1 = []
    if abs(p_t_s_3[2] - p_t_s_1[2]) * 180 / \
            pi > 90:  # if |saiC-saiA| > 90 degree, saiA + 180 degree, thetaA = -thetaA
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
                # find angle, Angular velocity, Angular acceleration
                q_x, q_v, q_a = utlis.straight_line(
                    x_y_z1[i], x_y_z2[i], cartesian_t)
                cartesian_x_sum.append(q_x)  # angle
                cartesian_v_sum.append(q_v)  # Angular velocity
                cartesian_a_sum.append(q_a)  # Angular acceleration

        elif 0.3 <= cartesian_t < 0.7:  # transfer movement
            for i in range(3):
                q_x, q_v, q_a = utlis.transition(
                    x_y_z1[i], x_y_z2[i], x_y_z3[i], cartesian_t - 0.5)
                cartesian_x_sum.append(q_x)
                cartesian_v_sum.append(q_v)
                cartesian_a_sum.append(q_a)

        elif 0.7 <= cartesian_t:  # linear movement
            c = cartesian_t - 0.5
            for i in range(3):
                q_x, q_v, q_a = utlis.straight_line(x_y_z2[i], x_y_z3[i], c)
                cartesian_x_sum.append(q_x)
                cartesian_v_sum.append(q_v)
                cartesian_a_sum.append(q_a)
        cartesian_t_x.append(cartesian_t)
        cartesian_t += 0.002

    # prepare memory to store x, y, z, their Angular velocity, Angular
    # acceleration
    cartesian_x_p = []
    cartesian_x_v = []
    cartesian_x_a = []

    cartesian_y_p = []
    cartesian_y_v = []
    cartesian_y_a = []

    cartesian_z_p = []
    cartesian_z_v = []
    cartesian_z_a = []

    # save theta1 to theta6 angle, Angular velocity, Angular acceleration
    for i in range(1503):
        if i % 3 == 0:
            cartesian_x_p.append(cartesian_x_sum[i] * 100)
            cartesian_x_v.append(cartesian_v_sum[i] * 100)
            cartesian_x_a.append(cartesian_a_sum[i] * 100)

        if i % 3 == 1:
            cartesian_y_p.append(cartesian_x_sum[i] * 100)
            cartesian_y_v.append(cartesian_v_sum[i] * 100)
            cartesian_y_a.append(cartesian_a_sum[i] * 100)

        if i % 3 == 2:
            cartesian_z_p.append(cartesian_x_sum[i] * 100)
            cartesian_z_v.append(cartesian_v_sum[i] * 100)
            cartesian_z_a.append(cartesian_a_sum[i] * 100)

    utlis.plot_position(
        cartesian_t_x, [
            cartesian_x_p, cartesian_y_p, cartesian_z_p])
    # x, y, z Angular velocity
    utlis.plot_position(cartesian_t_x,
                        [cartesian_x_p,
                         cartesian_y_p,
                         cartesian_z_p],
                        y_label='Velocity(cm/s)',
                        title='velocity')
    # x, y, z Angular velocity
    utlis.plot_position(cartesian_t_x,
                        [cartesian_x_a,
                         cartesian_y_a,
                         cartesian_z_a],
                        y_label='Acceleration(cm/s)',
                        title='acceleration')
    plt.show()

    # print 3D picture
    ax = plt.subplot(projection='3d')
    ax.scatter(x, y, z, c='b')

    text = str(round(x[0], 4)) + ', ' + \
        str(round(y[0], 4)) + ', ' + str(round(z[0], 4))
    ax.text(x[0], y[0], z[0], text, fontsize=10)
    text = str(round(x[250], 4)) + ', ' + \
        str(round(y[250], 4)) + ', ' + str(round(z[250], 4))
    ax.text(x[250], y[250], z[250], text, fontsize=10)
    text = str(round(x[500], 4)) + ', ' + \
        str(round(y[500], 4)) + ', ' + str(round(z[500], 4))
    ax.text(x[500], y[500], z[500], text, fontsize=10)

    ax.set_zlabel('Z')
    ax.set_ylabel('Y')
    ax.set_xlabel('X')

    # coordinate A in 3D
    a = utlis.Arrow3D([x[0], x[0]], [y[0], y[0]], [z[0], z[0] -
                                                   10], mutation_scale=20, lw=2, arrowstyle="->", color="b")
    b = utlis.Arrow3D([x[0], x[0] + 10], [y[0], y[0]], [z[0], z[0]],
                      mutation_scale=20, lw=2, arrowstyle="->", color="r")
    c = utlis.Arrow3D([x[0], x[0]], [y[0], y[0] - 10], [z[0], z[0]],
                      mutation_scale=20, lw=2, arrowstyle="->", color="g")
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)

    # coordinate B in 3D
    a = utlis.Arrow3D([x[250], x[250] - 10], [y[250], y[250]], [z[250],
                                                                z[250]], mutation_scale=20, lw=2, arrowstyle="->", color="b")
    b = utlis.Arrow3D([x[250], x[250]], [y[250], y[250] - 5], [z[250],
                                                               z[250]], mutation_scale=20, lw=2, arrowstyle="->", color="r")
    c = utlis.Arrow3D([x[250], x[250]], [y[250], y[250]], [
                      z[250], z[250] + 10], mutation_scale=20, lw=2, arrowstyle="->", color="g")
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)

    # coordinate C in 3D
    a = utlis.Arrow3D([x[500], x[500] + 10], [y[500], y[500]], [z[500],
                                                                z[500]], mutation_scale=20, lw=2, arrowstyle="->", color="b")
    b = utlis.Arrow3D([x[500], x[500]], [y[500], y[500]], [
                      z[500], z[500] + 10], mutation_scale=20, lw=2, arrowstyle="->", color="r")
    c = utlis.Arrow3D([x[500], x[500]], [y[500], y[500] - 5], [z[500],
                                                               z[500]], mutation_scale=20, lw=2, arrowstyle="->", color="g")
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)

    plt.show()

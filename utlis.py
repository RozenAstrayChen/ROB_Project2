from matplotlib.patches import FancyArrowPatch  # draw a vector
from mpl_toolkits.mplot3d import proj3d  # draw 3D vector
import matplotlib.pyplot as plt  # show the picure


def judgeangle(Tns):
    '''
    judge angle is vaild or not
    '''
    ret = []
    count = 0
    for Tn in Tns:
        flag = False
        count += 1
        if Tn[0] >= 160 or Tn[0] <= -160:
            #print('solution ', count, ' theta 1 is out of range !\n')
            # print(Tn,'\n')
            flag = True
        elif Tn[1] >= 125 or Tn[1] <= -125:
            #print('solution ', count, ' theta 2 is out of range !\n')
            # print(Tn,'\n')
            flag = True
        elif Tn[2] >= 135 or Tn[2] <= -135:
            #print('solution ', count, ' theta 3 is out of range !')
            # print(Tn,'\n')
            flag = True
        elif Tn[3] >= 140 or Tn[3] <= -140:
            #print('solution ', count, ' theta 4 is out of range !')
            # print(Tn,'\n')
            flag = True
        elif Tn[4] >= 100 or Tn[4] <= -100:
            #print('solution ', count, ' theta 5 is out of range !')
            # print(Tn,'\n')
            flag = True
        elif Tn[5] >= 260 or Tn[5] <= -260:
            #print('solution ', count, ' theta 6 is out of range !')
            # print(Tn,'\n')
            flag = True

        elif flag == False:
            #print('solution ', count, 'is vaild')
            # print(Tn,'\n')
            ret.append(Tn)
    return ret


def collect_angle():
    '''
    let user input angle and check angle is vaild or not
    '''
    t1 = -1000
    t2 = -1000
    t3 = -1000
    t4 = -1000
    t5 = -1000
    t6 = -1000
    while t1 >= 160 or t1 <= -160:
        t1 = int(input('pls input theta1: '))
    while t2 >= 160 or t2 <= -160:
        t2 = int(input('pls input theta2: '))
    while t3 >= 160 or t3 <= -160:
        t3 = int(input('pls input theta3: '))
    while t4 >= 160 or t4 <= -160:
        t4 = int(input('pls input theta4: '))
    while t5 >= 160 or t5 <= -160:
        t5 = int(input('pls input theta5: '))
    while t6 >= 160 or t6 <= -160:
        t6 = int(input('pls input theta6: '))

    return [t1, t2, t3, t4, t5, t6]


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
    h = t / T
    q_x = (thetaB - thetaA) * h + thetaA
    q_v = (thetaB - thetaA) / T
    q_a = 0
    return q_x, q_v, q_a

# transition portion


def transition(thetaA, thetaB, thetaC, t):
    tacc = 0.2
    T = 0.5
    h = (t + tacc) / (2 * tacc)
    deltaC = thetaC - thetaB
    deltaB = 0.4 * thetaA - 0.4 * thetaB
    q_x = (((deltaC * tacc / T) + deltaB) * (2 - h)
           * h * h - 2 * deltaB) * h + thetaB + deltaB
    q_v = (((deltaC * tacc / T) + deltaB) *
           (1.5 - h) * 2 * h * h - deltaB) / tacc
    q_a = ((deltaC * tacc / T) + deltaB) * (1 - h) * 3 * h / tacc / tacc
    return q_x, q_v, q_a


def plot(t_x, joint_thetas, x_label='time', y_label='Angle(°)'):
    plt.figure()
    plt.subplot(3, 2, 1)
    plt.scatter(t_x, joint_thetas[0], marker='.')
    plt.ylabel(y_label)
    plt.title("joint 1")

    # joint 2
    plt.subplot(3, 2, 2)
    plt.scatter(t_x, joint_thetas[1], marker='.')
    plt.ylabel(y_label)
    plt.title("joint 2")

    # joint 3
    plt.subplot(3, 2, 3)
    plt.scatter(t_x, joint_thetas[2], marker='.')
    plt.ylabel(y_label)
    plt.title("joint 3")

    # joint 4
    plt.subplot(3, 2, 4)
    plt.scatter(t_x, joint_thetas[3], marker='.')
    plt.ylabel(y_label)
    plt.title("joint 4")

    # joint 5
    plt.subplot(3, 2, 5)
    plt.scatter(t_x, joint_thetas[4], marker='.')
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title("joint 5")

    # joint 6
    plt.subplot(3, 2, 6)
    plt.scatter(t_x, joint_thetas[5], marker='.')
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title("joint 6")

    # plt.savefig("score.png")


def plot_position(
        t_x,
        xyz,
        x_label='time',
        y_label='Place(cm)',
        title='position'):
    # cartesian_x
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.scatter(t_x, xyz[0], marker='.')

    plt.ylabel(y_label)
    plt.title(title + " of x")

    # cartesian_y
    plt.subplot(3, 1, 2)
    plt.scatter(t_x, xyz[1], marker='.')

    plt.ylabel(y_label)
    plt.title(title + " of y")

    # cartesian_z
    plt.subplot(3, 1, 3)
    plt.scatter(t_x, xyz[2], marker='.')

    plt.ylabel(y_label)
    plt.title(title + " of z")
    plt.xlabel(x_label)

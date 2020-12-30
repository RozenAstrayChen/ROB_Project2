from math import cos, sin, pi, atan2
import numpy as np
def getTable(d, a, alpha, theta):
    '''
    [d, a, ahpla, theta]
    return numpy array
    '''
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def getPUMA560(input=None):
    '''
    numyp array is define by Joint d(m) a(m) alpha theta
    you have two options. No any input is default angle 50,
    input is array type
    return 6x4 arrself.ay
    '''
    if input == None:
        '''default angle is 50'''
        return np.array([
            [0, 0, -90/180*pi, 50/180*pi],
            [0, 0.432, 0, 50/180*pi],
            [0.149, -0.02, 90/180*pi, 50/180*pi],
            [0.433, 0, -90/180*pi, 50/180*pi],
            [0, 0, 90/180*pi, 50/180*pi],
            [0, 0, 0, 50/180*pi]
        ])
    else:
        return np.array([
            [0, 0, -90 / 180 * pi, input[0] / 180 * pi],
            [0, 0.432, 0, input[0] / 180 * pi],
            [0.149, -0.02, 90 / 180 * pi, input[0] / 180 * pi],
            [0.433, 0, -90 / 180 * pi, input[0] / 180 * pi],
            [0, 0, 90 / 180 * pi, input[0] / 180 * pi],
            [0, 0, 0, input[0] / 180 * pi]
        ])

class inverseKinematics():
    '''
    This is inverse kinematices class
    The final want to find theta 1-6
    and the order is 1 -> 3 -> 2 -> 4 -> 6 -> 5
    '''
    def __init__(self, Tn):
        '''
        Constructing by noap table
        '''
        self.d3 = 0.149; self.d4 = 0.433;
        self.a2 = 0.432; self.a3 = -0.02;

        self.px = Tn[0][3]; self.ax = Tn[0][2]; self.ox = Tn[0][1]
        self.nx = Tn[0][0]; self.py = Tn[1][3]; self.ay = Tn[1][2]
        self.oy = Tn[1][1]; self.ny = Tn[1][0]; self.pz = Tn[2][3]
        self.az = Tn[2][2]; self.oz = Tn[2][1]; self.nz = Tn[2][0]
        self.p1 = (self.px ** 2 + self.py ** 2) ** 0.5
        self.p3 = (self.d4 ** 2 + self.a3 ** 2) ** 0.5
        self.M = (self.px ** 2 + self.py ** 2 + self.pz ** 2 - self.a2 ** 2 - self.a3 ** 2 - self.d4 ** 2 - self.d3 ** 2) / (2 * self.a2)

    def findTheta_1(self):
        self.theta1a = (atan2(self.py, self.px) - atan2(self.d3 / self.p1, (1 - (self.d3 / self.p1) ** 2) ** 0.5))
        self.theta1b = (atan2(self.py, self.px) - atan2(self.d3 / self.p1, -(1 - (self.d3 / self.p1) ** 2) ** 0.5))

        return [self.theta1a, self.theta1b]

    def findTheta_3(self):
        self.theta3a = (-atan2(self.a3, self.d4) + atan2(self.M / self.p3, (1 - (self.M / self.p3) ** 2) ** 0.5))
        self.theta3b = (-atan2(self.a3, self.d4) + atan2(self.M / self.p3, -(1 - (self.M / self.p3) ** 2) ** 0.5))

        return [self.theta3a, self.theta3b]

    def findTheta_2(self):
        self.theta2aa = atan2(
            -(self.a3 + self.a2 * cos(self.theta3a)) * self.pz + (cos(self.theta1a) * self.px + sin(self.theta1a) * self.py) * (self.a2 * sin(self.theta3a) + self.d4)
            , (self.d4 + self.a2 * sin(self.theta3a)) * self.pz + (cos(self.theta1a) * self.px + sin(self.theta1a) * self.py) * (
                        self.a2 * cos(self.theta3a) + self.a3)) - self.theta3a
        self.theta2ab = atan2(
            -(self.a3 + self.a2 * cos(self.theta3b)) * self.pz + (cos(self.theta1a) * self.px + sin(self.theta1a) * self.py) * (self.a2 * sin(self.theta3b) + self.d4)
            , (self.d4 + self.a2 * sin(self.theta3b)) * self.pz + (cos(self.theta1a) * self.px + sin(self.theta1a) * self.py) * (
                        self.a2 * cos(self.theta3b) + self.a3)) - self.theta3b
        self.theta2ba = atan2(
            -(self.a3 + self.a2 * cos(self.theta3a)) * self.pz + (cos(self.theta1b) * self.px + sin(self.theta1b) * self.py) * (self.a2 * sin(self.theta3a) + self.d4)
            , (self.d4 + self.a2 * sin(self.theta3a)) * self.pz + (cos(self.theta1b) * self.px + sin(self.theta1b) * self.py) * (
                        self.a2 * cos(self.theta3a) + self.a3)) - self.theta3a
        self.theta2bb = atan2(
            -(self.a3 + self.a2 * cos(self.theta3b)) * self.pz + (cos(self.theta1b) * self.px + sin(self.theta1b) * self.py) * (self.a2 * sin(self.theta3b) + self.d4)
            , (self.d4 + self.a2 * sin(self.theta3b)) * self.pz + (cos(self.theta1b) * self.px + sin(self.theta1b) * self.py) * (
                        self.a2 * cos(self.theta3b) + self.a3)) - self.theta3b

        return [self.theta2aa, self.theta2ab, self.theta2ba, self.theta2bb]

    def findTheta_4(self):
        self.theta4aap = atan2(-self.ax * sin(self.theta1a) + cos(self.theta1a) * self.ay,
                          self.ax * cos(self.theta1a) * cos(self.theta2aa + self.theta3a) + sin(self.theta1a) * cos(
                              self.theta2aa + self.theta3a) * self.ay - sin(
                              self.theta2aa + self.theta3a) * self.az)
        self.theta4aan = atan2(self.ax * sin(self.theta1a) - cos(self.theta1a) * self.ay,
                          -self.ax * cos(self.theta1a) * cos(self.theta2aa + self.theta3a) - sin(self.theta1a) * cos(
                              self.theta2aa + self.theta3a) * self.ay + sin(
                              self.theta2aa + self.theta3a) * self.az)
        self.theta4abp = atan2(-self.ax * sin(self.theta1a) + cos(self.theta1a) * self.ay,
                          self.ax * cos(self.theta1a) * cos(self.theta2ab + self.theta3b) + sin(self.theta1a) * cos(
                              self.theta2ab + self.theta3b) * self.ay - sin(
                              self.theta2ab + self.theta3b) * self.az)
        self.theta4abn = atan2(self.ax * sin(self.theta1a) - cos(self.theta1a) * self.ay,
                          -self.ax * cos(self.theta1a) * cos(self.theta2ab + self.theta3b) - sin(self.theta1a) * cos(
                              self.theta2ab + self.theta3b) * self.ay + sin(
                              self.theta2ab + self.theta3b) * self.az)
        self.theta4bap = atan2(-self.ax * sin(self.theta1b) + cos(self.theta1b) * self.ay,
                          self.ax * cos(self.theta1b) * cos(self.theta2ba + self.theta3a) + sin(self.theta1b) * cos(
                              self.theta2ba + self.theta3a) * self.ay - sin(
                              self.theta2ba + self.theta3a) * self.az)
        self.theta4ban = atan2(self.ax * sin(self.theta1b) - cos(self.theta1b) * self.ay,
                          -self.ax * cos(self.theta1b) * cos(self.theta2ba + self.theta3a) - sin(self.theta1b) * cos(
                              self.theta2ba + self.theta3a) * self.ay + sin(
                              self.theta2ba + self.theta3a) * self.az)
        self.theta4bbp = atan2(-self.ax * sin(self.theta1b) + cos(self.theta1b) * self.ay,
                          self.ax * cos(self.theta1b) * cos(self.theta2bb + self.theta3b) + sin(self.theta1b) * cos(
                              self.theta2bb + self.theta3b) * self.ay - sin(
                              self.theta2bb + self.theta3b) * self.az)
        self.theta4bbn = atan2(self.ax * sin(self.theta1b) - cos(self.theta1b) * self.ay,
                          -self.ax * cos(self.theta1b) * cos(self.theta2bb + self.theta3b) - sin(self.theta1b) * cos(
                              self.theta2bb + self.theta3b) * self.ay + sin(
                              self.theta2bb + self.theta3b) * self.az)

        return [self.theta4aap, self.theta4aan, self.theta4abp, self.theta4abn, self.theta4bap, self.theta4ban, self.theta4bbp, self.theta4bbn]

    def findTheta_6(self):
        self.theta6aan = atan2(
            -cos(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.ox - sin(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.oy - cos(
                self.theta2aa + self.theta3a) * self.oz
            , cos(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.nx + sin(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.ny + cos(
                self.theta2aa + self.theta3a) * self.nz)
        self.theta6aap = atan2(
            cos(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.ox + sin(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.oy + cos(
                self.theta2aa + self.theta3a) * self.oz
            ,
            -cos(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.nx - sin(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.ny - cos(
                self.theta2aa + self.theta3a) * self.nz)
        self.theta6abn = atan2(
            -cos(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.ox - sin(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.oy - cos(
                self.theta2ab + self.theta3b) * self.oz
            , cos(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.nx + sin(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.ny + cos(
                self.theta2ab + self.theta3b) * self.nz)
        self.theta6abp = atan2(
            cos(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.ox + sin(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.oy + cos(
                self.theta2ab + self.theta3b) * self.oz
            ,
            -cos(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.nx - sin(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.ny - cos(
                self.theta2ab + self.theta3b) * self.nz)
        self.theta6ban = atan2(
            -cos(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.ox - sin(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.oy - cos(
                self.theta2ba + self.theta3a) * self.oz
            , cos(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.nx + sin(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.ny + cos(
                self.theta2ba + self.theta3a) * self.nz)
        self.theta6bap = atan2(
            cos(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.ox + sin(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.oy + cos(
                self.theta2ba + self.theta3a) * self.oz
            ,
            -cos(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.nx - sin(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.ny - cos(
                self.theta2ba + self.theta3a) * self.nz)
        self.theta6bbn = atan2(
            -cos(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.ox - sin(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.oy - cos(
                self.theta2bb + self.theta3b) * self.oz
            , cos(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.nx + sin(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.ny + cos(
                self.theta2bb + self.theta3b) * self.nz)
        self.theta6bbp = atan2(
            cos(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.ox + sin(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.oy + cos(
                self.theta2bb + self.theta3b) * self.oz
            , -cos(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.nx - sin(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.ny - cos(
                self.theta2bb + self.theta3b) * self.nz)

        return [self.theta6aan, self.theta6aap, self.theta6abn, self.theta6abp, self.theta6ban, self.theta6bap, self.theta6bbn, self.theta6bbp]

    def findTheta_5(self):
        self.theta5aap = atan2(
            (cos(self.theta1a) * cos(self.theta4aap) * cos(self.theta2aa + self.theta3a) - sin(self.theta1a) * sin(self.theta4aap)) * self.ax + (
                    cos(self.theta1a) * sin(self.theta4aap) + cos(self.theta4aap) * cos(self.theta2aa + self.theta3a) * sin(self.theta1a)) * self.ay - cos(
                self.theta4aap) * sin(
                self.theta2aa + self.theta3a) * self.az, cos(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.ax + sin(self.theta1a) * sin(
                self.theta2aa + self.theta3a) * self.ay + cos(self.theta2aa + self.theta3a) * self.az)

        self.theta5aan = atan2(
            (cos(self.theta1a) * cos(self.theta4aan) * cos(self.theta2aa + self.theta3a) - sin(self.theta1a) * sin(self.theta4aan)) * self.ax + (
                    cos(self.theta1a) *
                    sin(self.theta4aan) + cos(self.theta4aan) * cos(self.theta2aa + self.theta3a) * sin(self.theta1a)) * self.ay - cos(
                self.theta4aan) * sin(
                self.theta2aa + self.theta3a) * self.az, cos(self.theta1a) * sin(self.theta2aa + self.theta3a) * self.ax + sin(self.theta1a) * sin(
                self.theta2aa + self.theta3a) * self.ay + cos(self.theta2aa + self.theta3a) * self.az)
        self.theta5abp = atan2(
            (cos(self.theta1a) * cos(self.theta4abp) * cos(self.theta2ab + self.theta3b) - sin(self.theta1a) * sin(self.theta4abp)) * self.ax + (
                    cos(self.theta1a) *
                    sin(self.theta4abp) + cos(self.theta4abp) * cos(self.theta2ab + self.theta3b) * sin(self.theta1a)) * self.ay - cos(
                self.theta4abp) * sin(
                self.theta2ab + self.theta3b) * self.az, cos(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.ax + sin(self.theta1a) * sin(
                self.theta2ab + self.theta3b) * self.ay + cos(self.theta2ab + self.theta3b) * self.az)
        self.theta5abn = atan2(
            (cos(self.theta1a) * cos(self.theta4abn) * cos(self.theta2ab + self.theta3b) - sin(self.theta1a) * sin(self.theta4abn)) * self.ax + (
                    cos(self.theta1a) *
                    sin(self.theta4abn) + cos(self.theta4abn) * cos(self.theta2ab + self.theta3b) * sin(self.theta1a)) * self.ay - cos(
                self.theta4abn) * sin(
                self.theta2ab + self.theta3b) * self.az, cos(self.theta1a) * sin(self.theta2ab + self.theta3b) * self.ax + sin(self.theta1a) * sin(
                self.theta2ab + self.theta3b) * self.ay + cos(self.theta2ab + self.theta3b) * self.az)
        self.theta5bap = atan2(
            (cos(self.theta1b) * cos(self.theta4bap) * cos(self.theta2ba + self.theta3a) - sin(self.theta1b) * sin(self.theta4bap)) * self.ax + (
                    cos(self.theta1b) *
                    sin(self.theta4bap) + cos(self.theta4bap) * cos(self.theta2ba + self.theta3a) * sin(self.theta1b)) * self.ay - cos(
                self.theta4bap) * sin(
                self.theta2ba + self.theta3a) * self.az, cos(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.ax + sin(self.theta1b) * sin(
                self.theta2ba + self.theta3a) * self.ay + cos(self.theta2ba + self.theta3a) * self.az)
        self.theta5ban = atan2(
            (cos(self.theta1b) * cos(self.theta4ban) * cos(self.theta2ba + self.theta3a) - sin(self.theta1b) * sin(self.theta4ban)) * self.ax + (
                    cos(self.theta1b) *
                    sin(self.theta4ban) + cos(self.theta4ban) * cos(self.theta2ba + self.theta3a) * sin(self.theta1b)) * self.ay - cos(
                self.theta4ban) * sin(
                self.theta2ba + self.theta3a) * self.az, cos(self.theta1b) * sin(self.theta2ba + self.theta3a) * self.ax + sin(self.theta1b) * sin(
                self.theta2ba + self.theta3a) * self.ay + cos(self.theta2ba + self.theta3a) * self.az)
        self.theta5bbp = atan2(
            (cos(self.theta1b) * cos(self.theta4bbp) * cos(self.theta2bb + self.theta3b) - sin(self.theta1b) * sin(self.theta4bbp)) * self.ax + (
                    cos(self.theta1b) *
                    sin(self.theta4bbp) + cos(self.theta4bbp) * cos(self.theta2bb + self.theta3b) * sin(self.theta1b)) * self.ay - cos(
                self.theta4bbp) * sin(
                self.theta2bb + self.theta3b) * self.az, cos(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.ax + sin(self.theta1b) * sin(
                self.theta2bb + self.theta3b) * self.ay + cos(self.theta2bb + self.theta3b) * self.az)
        self.theta5bbn = atan2(
            (cos(self.theta1b) * cos(self.theta4bbn) * cos(self.theta2bb + self.theta3b) - sin(self.theta1b) * sin(self.theta4bbn)) * self.ax + (
                    cos(self.theta1b) *
                    sin(self.theta4bbn) + cos(self.theta4bbn) * cos(self.theta2bb + self.theta3b) * sin(self.theta1b)) * self.ay - cos(
                self.theta4bbn) * sin(
                self.theta2bb + self.theta3b) * self.az, cos(self.theta1b) * sin(self.theta2bb + self.theta3b) * self.ax + sin(self.theta1b) * sin(
                self.theta2bb + self.theta3b) * self.ay + cos(self.theta2bb + self.theta3b) * self.az)

        return [self.theta5aap, self.theta5aan, self.theta5abp, self.theta5abn, self.theta5bap, self.theta5ban, self.theta5bbp, self.theta5bbn]

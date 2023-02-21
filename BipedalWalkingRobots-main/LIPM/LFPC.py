# -*-coding:UTF-8 -*-
import numpy as np


class LFPC:
    def __init__(self,
                 dt=0.001,
                 T_sup=1.0,
                 h=1.0,
                 support_leg='left_leg'):
        # initial params in LFPC
        self.T_sup = T_sup
        self.h = h
        self.dt = dt
        self.t = 0.0
        self.b = 0.3
        self.support_leg = support_leg

        p_x = 0
        p_y = 0

        # COM initial state
        self.x_0 = 0
        self.vx_0 = 0
        self.y_0 = 0
        self.vy_0 = 0
        # COM real-time state
        self.x_t = 0
        self.vx_t = 0
        self.y_t = 0
        self.vy_t = 0

        # foot location in gobal
        self.p_x1 = 0.0
        self.p_y1 = 0.0
        self.p_x2 = 0.0
        self.p_y2 = 0.0

        # control params
        self.al = 0.0
        self.aw = 0.0
        self.theta = 0.0

        #这里的为全局的坐标
        self.left_foot_pos = [0.0, 0.0, 0.0]
        self.right_foot_pos = [0.0, 0.0, 0.0]
        self.COM_pos = [0.0, 0.0, 0.0]
        self.COM_pos[2] = h

    def initializeModel(self, COM_pos, left_foot_pos, right_foot_pos):
        self.COM_pos = COM_pos
        self.left_foot_pos = left_foot_pos
        self.right_foot_pos = right_foot_pos

        self.zc = self.COM_pos[2]
        self.T_c = np.sqrt(self.zc / 10)  # set gravity parameter as 9.8
        self.C = np.cosh(self.T_sup/self.T_c)
        self.S = np.sinh(self.T_sup/self.T_c)

    def SetCtrlParams(self, al, aw, theta):
        self.al = al
        self.aw = aw
        self.theta = theta

    def step(self):
        self.t += self.dt
        t = self.t
        self.x_t, self.vx_t, self.y_t, self.vy_t = self.calculateXtVt(t)

    def calculateXtVt(self, t):
        T_c = self.T_c
        # calculate x
        x_t = self.x_0 * np.cosh(t/T_c) + T_c * self.vx_0 * np.sinh(t/T_c)
        vx_t = self.x_0/T_c * np.sinh(t/T_c) + self.vx_0 * np.cosh(t/T_c)
        # calculate y
        y_t = self.y_0 * np.cosh(t/T_c) + T_c * self.vy_0 * np.sinh(t/T_c)
        vy_t = self.y_0/T_c * np.sinh(t/T_c) + self.vy_0 * np.cosh(t/T_c)

        return x_t, vx_t, y_t, vy_t

    def updateLFPC(self, vx, vy):
        al = self.al
        aw = self.aw
        theta = self.theta
        b = self.b

        xf1 = -al * np.cos(theta) + aw * np.sin(theta)+b * vx
        xf2 = -al * np.cos(theta) - aw * np.sin(theta)+b * vx
        yf1 = -al * np.sin(theta) - aw * np.cos(theta)+b * vy
        yf2 = -al * np.sin(theta) + aw * np.cos(theta)+b * vy

        return xf1, xf2, yf1, yf2

    def calculateFinalSate(self):
        T_sup = self.T_sup
        x_d, vx_d, y_d, vy_d = self.calculateXtVt(T_sup)
        return x_d, vx_d, y_d, vy_d

    def updateNextFootLocation(self):
        x_d, vx_d, y_d, vy_d = self.calculateFinalSate()
        xf1, xf2, yf1, yf2 = self.updateLFPC(vx_d,vy_d)
        self.p_x1 = self.COM_pos[0] + xf1
        self.p_x2 = self.COM_pos[0] + xf2 
        self.p_y1 = self.COM_pos[1] + yf1
        self.p_y2 = self.COM_pos[1] + yf2 
        return

    def switchSupportLeg(self):

        xf1, xf2, yf1, yf2 = self.updateLFPC(self.vx_0,self.vy_0)


        if self.support_leg is 'left_leg':
            print('\n---- switch the support leg to the right leg')
            self.support_leg = 'right_leg'
            self.x_0 = -xf1
            self.y_0 = -yf1
        elif self.support_leg is 'right_leg':
            print('\n---- switch the support leg to the left leg')
            self.support_leg = 'left_leg'
            self.x_0 = -xf2
            self.y_0 = -yf2

        x_d, vx_d, y_d, vy_d = self.calculateFinalSate()
        self.t = 0
        self.vx_0 = vx_d
        self.vy_0 = vy_d        

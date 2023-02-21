from matplotlib import gridspec
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# <--- This is important for 3d plotting
from mpl_toolkits.mplot3d import Axes3D


from LIPM_3D import LIPM3D
from LFPC import LFPC


class Ball:
    def __init__(self, size=10, shape='o'):
        self.scatter, = ax.plot(
            [], [], [], shape, markersize=size, animated=True)

    def update(self, pos):
        # draw ball
        self.scatter.set_data_3d(pos)


class Line:
    def __init__(self, size=1, color='g'):
        self.line, = ax.plot([], [], [], linewidth=size,
                             color=color, animated=True)

    def update(self, pos):
        # draw line
        self.line.set_xdata(pos[0, :])
        self.line.set_ydata(pos[1, :])
        self.line.set_3d_properties(np.asarray(pos[2, :]))


class LIPM_3D_Animate():
    def __init__(self):
        self.origin = Ball(size=2, shape='ko')
        self.COM = Ball(size=16, shape='ro')
        self.COM_trajectory = Line(size=1, color='g')
        self.COM_head = Ball(size=2, shape='ro')

        self.left_foot = Ball(size=5, shape='bo')
        self.right_foot = Ball(size=5, shape='mo')

        self.left_leg = Line(size=3, color='b')
        self.right_leg = Line(size=3, color='m')

    def update(self, COM_pos, COM_pos_trajectory, left_foot_pos, right_foot_pos):
        self.origin.update([0, 0, 0])
        self.COM.update(COM_pos)
        self.COM_trajectory.update(COM_pos_trajectory)
        self.COM_head.update(COM_pos_trajectory[:, -1])

        self.left_foot.update(left_foot_pos)
        self.right_foot.update(right_foot_pos)

        pos_1 = np.zeros((3, 2))
        pos_1[:, 0] = COM_pos
        pos_1[:, 1] = left_foot_pos
        self.left_leg.update(pos_1)

        pos_2 = np.zeros((3, 2))
        pos_2[:, 0] = COM_pos
        pos_2[:, 1] = right_foot_pos
        self.right_leg.update(pos_2)

        artists = []
        artists.append(self.origin.scatter)
        artists.append(self.left_leg.line)
        artists.append(self.right_leg.line)
        artists.append(self.COM.scatter)
        artists.append(self.COM_trajectory.line)
        artists.append(self.COM_head.scatter)
        artists.append(self.left_foot.scatter)
        artists.append(self.right_foot.scatter)

        # automatic set the x, y view limitation
        if COM_pos[0] >= 3.0:
            ax.set_xlim(-1.0 + COM_pos[0] - 3.0, 4.0 + COM_pos[0] - 3.0)
        elif COM_pos[0] <= 0:
            ax.set_xlim(-1.0 + COM_pos[0], 4.0 + COM_pos[0])

        if COM_pos[1] >= 1.5:
            ax.set_ylim(-2 + COM_pos[1] - 1.5, 2 + COM_pos[1] - 1.5)
        elif COM_pos[1] <= -1.5:
            ax.set_ylim(-2 + COM_pos[1] + 1.5, 2 + COM_pos[1] + 1.5)

        ax.set_zlim(-0.01, 2.0)

        artists.append(ax)

        return artists


def ani_3D_init():
    return []


def ani_3D_update(i):
    COM_pos = [COM_pos_x[i], COM_pos_y[i], 1]
    COM_pos_trajectory = np.zeros((3, i))
    COM_pos_trajectory[0, :] = COM_pos_x[0:i]
    COM_pos_trajectory[1, :] = COM_pos_y[0:i]
    COM_pos_trajectory[2, :] = np.zeros((1, i))

    left_foot_pos = [left_foot_pos_x[i],
                     left_foot_pos_y[i], left_foot_pos_z[i]]
    right_foot_pos = [right_foot_pos_x[i],
                      right_foot_pos_y[i], right_foot_pos_z[i]]

    artists = LIPM_3D_ani.update(
        COM_pos, COM_pos_trajectory, left_foot_pos, right_foot_pos)

    return artists


def ani_2D_init():
    COM_traj_ani.set_data(COM_pos_x[0:0], COM_pos_y[0:0])
    COM_pos_ani.set_data(COM_pos_x[0], COM_pos_y[0])
    left_foot_pos_ani.set_data(left_foot_pos_x[0], left_foot_pos_y[0])
    right_foot_pos_ani.set_data(right_foot_pos_x[0], right_foot_pos_y[0])

    return [COM_pos_ani, COM_traj_ani, left_foot_pos_ani, right_foot_pos_ani]


def ani_2D_update(i):
    COM_traj_ani.set_data(COM_pos_x[0:i], COM_pos_y[0:i])
    COM_pos_ani.set_data(COM_pos_x[i], COM_pos_y[i])
    left_foot_pos_ani.set_data(left_foot_pos_x[i], left_foot_pos_y[i])
    right_foot_pos_ani.set_data(right_foot_pos_x[i], right_foot_pos_y[i])

    ani_text_COM_pos.set_text(COM_pos_str % (COM_pos_x[i], COM_pos_y[i]))

    # # automatic set the x, y view limitation
    # bx.set_xlim(-2.0 + COM_pos_x[i], 3.0 + COM_pos_x[i])
    # bx.set_ylim(-0.8 + COM_pos_y[i], 0.8 + COM_pos_y[i])

    return [COM_pos_ani, COM_traj_ani, left_foot_pos_ani, right_foot_pos_ani, ani_text_COM_pos, bx]


# %% ---------------------------------------------------------------- LIPM control
print('\n--------- Program start from here ...')

COM_pos_x = list()
COM_pos_y = list()
left_foot_pos_x = list()
left_foot_pos_y = list()
left_foot_pos_z = list()
right_foot_pos_x = list()
right_foot_pos_y = list()
right_foot_pos_z = list()

# Initialize the COM position and velocity
COM_pos_0 = [0, 0, 1.0]
COM_v0 = [0, 0]

# Initialize the foot positions
left_foot_pos = [0, 0, 0]
right_foot_pos = [0, 0, 0]

delta_t = 0.02

al = 0.4
aw = 0.1
theta = 0.0


LFPC_model = LFPC(dt=delta_t, T_sup=0.3)
LFPC_model.initializeModel(COM_pos_0, left_foot_pos, right_foot_pos)
LFPC_model.SetCtrlParams(al, aw, theta)

swing_data_len = int(LFPC_model.T_sup/delta_t)
swing_foot_pos = np.zeros((swing_data_len, 3))

# set the support leg to right leg in next step
LFPC_model.support_leg = 'left_leg'
if LFPC_model.support_leg == 'left_leg':
    support_foot_pos = LFPC_model.left_foot_pos
else:
    support_foot_pos = LFPC_model.right_foot_pos


LFPC_model.x_0 = LFPC_model.COM_pos[0] - support_foot_pos[0]
LFPC_model.y_0 = LFPC_model.COM_pos[1] - support_foot_pos[1]
LFPC_model.vx_0 = COM_v0[0]
LFPC_model.vy_0 = COM_v0[1]

step_num = 0
global_time = 0
total_time = 2.4  # seconds

j = 0

switch_index = swing_data_len

for i in range(int(total_time/delta_t)):
    # update time
    global_time += delta_t

    # lfpc step
    LFPC_model.step()

    if step_num >= 1:
        if LFPC_model.support_leg == 'left_leg':
            LFPC_model.right_foot_pos = [
                swing_foot_pos[j, 0], swing_foot_pos[j, 1], swing_foot_pos[j, 2]]
        else:
            LFPC_model.left_foot_pos = [
                swing_foot_pos[j, 0], swing_foot_pos[j, 1], swing_foot_pos[j, 2]]
        j += 1

    # record data
    LFPC_model.COM_pos[0] = LFPC_model.x_t + support_foot_pos[0]
    COM_pos_x.append(LFPC_model.COM_pos[0])
    LFPC_model.COM_pos[1] = LFPC_model.y_t + support_foot_pos[1]
    COM_pos_y.append(LFPC_model.COM_pos[1])

    print('\nglobal_time:', global_time, 'x:',
          LFPC_model.COM_pos[0], 'y:', LFPC_model.COM_pos[1])

    left_foot_pos_x.append(LFPC_model.left_foot_pos[0])
    left_foot_pos_y.append(LFPC_model.left_foot_pos[1])
    left_foot_pos_z.append(LFPC_model.left_foot_pos[2])
    right_foot_pos_x.append(LFPC_model.right_foot_pos[0])
    right_foot_pos_y.append(LFPC_model.right_foot_pos[1])
    right_foot_pos_z.append(LFPC_model.right_foot_pos[2])

    # switch the support leg
    if (i > 0) and (i % switch_index == 0):
        # change contorl param
        step_num += 1
        if step_num >= 5:  # stop forward after 5 steps
            LFPC_model.SetCtrlParams(al, aw, theta)

        if step_num >= 10:
            LFPC_model.SetCtrlParams(al, aw, theta)

        # update support_foot_pos
        j = 0
        LFPC_model.switchSupportLeg()  # switch the support leg
        if LFPC_model.support_leg == 'left_leg':
            support_foot_pos = LFPC_model.left_foot_pos
        else:
            support_foot_pos = LFPC_model.right_foot_pos

        # calculate the next foots locations
        LFPC_model.updateNextFootLocation()

        # calculate the foot positions for swing phase
        if LFPC_model.support_leg == 'left_leg':
            right_foot_target_pos = [LFPC_model.p_x1, LFPC_model.p_y1, 0]
            swing_foot_pos[:, 0] = np.linspace(
                LFPC_model.right_foot_pos[0], right_foot_target_pos[0], swing_data_len)
            swing_foot_pos[:, 1] = np.linspace(
                LFPC_model.right_foot_pos[1], right_foot_target_pos[1], swing_data_len)
            swing_foot_pos[1:swing_data_len-1, 2] = 0.1
        else:
            left_foot_target_pos = [LFPC_model.p_x2, LFPC_model.p_y2, 0]
            swing_foot_pos[:, 0] = np.linspace(
                LFPC_model.left_foot_pos[0], left_foot_target_pos[0], swing_data_len)
            swing_foot_pos[:, 1] = np.linspace(
                LFPC_model.left_foot_pos[1], left_foot_target_pos[1], swing_data_len)
            swing_foot_pos[1:swing_data_len-1, 2] = 0.1


# ------------------------------------------------- animation plot
data_len = len(COM_pos_x)
print('--------- plot')
fig = plt.figure(figsize=(8, 10))
spec = gridspec.GridSpec(nrows=2, ncols=1, height_ratios=[2.5, 1])
ax = fig.add_subplot(spec[0], projection='3d')
# ax.set_aspect('equal') # bugs
ax.set_xlim(-1.0, 4.0)
ax.set_ylim(-2.0, 2.0)
ax.set_zlim(-0.01, 2.0)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')

# view angles
ax.view_init(20, -150)
LIPM_3D_ani = LIPM_3D_Animate()
ani_3D = FuncAnimation(fig, ani_3D_update, frames=range(
    1, data_len), init_func=ani_3D_init, interval=1.0/delta_t, blit=True, repeat=True)
# ani_3D.save('./pic/LIPM_3D.gif', writer='imagemagick', fps=30)

bx = fig.add_subplot(spec[1], autoscale_on=False)
bx.set_xlim(-0.5, 5.0)
bx.set_ylim(-0.8, 0.8)
bx.set_aspect('equal')
bx.set_xlabel('x (m)')
bx.set_ylabel('y (m)')
bx.grid(ls='--')

COM_pos_str = 'COM = (%.2f, %.2f)'
ani_text_COM_pos = bx.text(0.05, 0.9, '', transform=bx.transAxes)


original_ani, = bx.plot(0, 0, marker='o', markersize=2, color='k')
left_foot_pos_ani, = bx.plot([], [], 'o', lw=2, color='b')
COM_traj_ani, = bx.plot(COM_pos_x[0], COM_pos_y[0], markersize=2, color='g')
COM_pos_ani, = bx.plot(
    COM_pos_x[0], COM_pos_y[0], marker='o', markersize=6, color='r')
left_foot_pos_ani, = bx.plot([], [], 'o', markersize=10, color='b')
right_foot_pos_ani, = bx.plot([], [], 'o', markersize=10, color='m')

ani_2D = FuncAnimation(fig=fig, init_func=ani_2D_init, func=ani_2D_update, frames=range(
    1, data_len), interval=1.0/delta_t, blit=True, repeat=True)
# ani_2D.save('./pic/COM_trajectory.gif', fps=30)

plt.show()
print('---------  Program terminated')

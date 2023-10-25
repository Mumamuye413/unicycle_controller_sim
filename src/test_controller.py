import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt

from utils.tools_plot_common import *

my_default_plt_setting()


# generate goal pose list from experiment
x_cord, y_cord = np.arange(5)*0.5, np.arange(5)*0.5
theta_cord = (np.arange(5)-2)*np.pi/4
g0 = np.zeros((3))
g1 = np.array([x_cord[1], y_cord[1], theta_cord[3]])
g2 = np.array([x_cord[2], y_cord[0], theta_cord[2]])
g3 = np.array([x_cord[2], y_cord[2], theta_cord[2]])
g4 = np.array([x_cord[3], y_cord[3], theta_cord[4]])
g5 = np.array([x_cord[4], y_cord[1], theta_cord[0]])
goal_list = [g1, g2, g3, g4, g5]
error_list = ['(0, -0,075, 3)', '(-0.065, 0.01, 0)', '(0.015, -0.085, 1)', 
              '(0.01, -0.04, -1)', '(-0.04, -0.03, 1)']
note_list = ['G1','G2','G3','G4','G5']


# initial figure
fig, ax = plt.subplots(figsize=(8,8))
ax.set_aspect('equal')
ax.grid('True')
ax.set_xlim([-0.5,2.5])
ax.set_ylim([-0.5,2])
# add initial pose marker
gmk = create_arrowhead_patch()
ax.plot(g0[0], g0[1], 'r', marker=gmk, ms=12)
ax.annotate('initial pose', xy=(g0[0]-0.1, g0[1]-0.15), fontsize='large')
# set up title and labels
ax.set_title('Low-level Controller Test \n Goal Poses and Controller Error')
ax.set_xlabel(r'${p_x}$ /m', loc='right', labelpad=4)
ax.set_ylabel(r'${p_y}$ /m', loc='top', labelpad=-8)

# add goal pose markers
for i in range(len(goal_list)):
    goal_pose  = goal_list[i]
    error = error_list[i]
    note = note_list[i]
    gmk = create_arrowhead_patch()
    gmk = gmk.transformed(mpl.transforms.Affine2D().rotate(goal_pose[2]))
    if i>0:
        scale = 1.414
        ax.plot(goal_pose[0], goal_pose[1], 'b', marker=gmk, ms=12*scale)  
    else:
        ax.plot(goal_pose[0], goal_pose[1], 'b', marker=gmk, ms=12)   
    ax.annotate(error, xy=(goal_pose[0]-0.1, goal_pose[1]-0.15), fontsize=12)
    ax.annotate(note, xy=(goal_pose[0]+0.05, goal_pose[1]+0.05), color='g', fontsize='large')
plt.show()
print('end')


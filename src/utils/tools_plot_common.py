#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 18:56:14 2020

Author: Zhichao Li at UCSD ERL
"""

import os
import numpy as np
import matplotlib as mpl
import matplotlib.patches as mp
from matplotlib.path import Path
from matplotlib.collections import PatchCollection
from matplotlib import pyplot as plt

# remove type3 fonts in figure
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42

#===========================================================
# change legend attributes
# ax11.legend(loc='center left', bbox_to_anchor=(1, 0.7), prop={'size':14})
# ax12.legend(loc='center left', bbox_to_anchor=(1, 0.5), prop={'size':14})


#===========================================================

def my_default_plt_setting():
    """ Set customized default setting.
    """
    import matplotlib as mpl
    mpl.rcParams['pdf.fonttype'] = 42
    mpl.rcParams['ps.fonttype'] = 42
    mpl.rcParams['lines.linewidth'] = 2.0
    mpl.rcParams['legend.fontsize'] = 'large'
    mpl.rcParams['axes.titlesize'] = 18
    mpl.rcParams['axes.labelsize'] = 14
    mpl.rcParams['axes.grid'] = True
    mpl.rcParams['axes.titlepad'] = 10
    # mpl.rcParams['axes.labelpad'] = 3
    plt.rcParams['figure.constrained_layout.use'] = True
    plt.rcParams["figure.constrained_layout.w_pad"] = 0.2
    plt.rcParams["figure.constrained_layout.h_pad"] = 0.1
    plt.rcParams["figure.constrained_layout.wspace"] = 0.05
    plt.rcParams["figure.constrained_layout.hspace"] = 0.04
    return 0


def save_fig_to_folder(fig, folder, fname, dpi=300, ftype_ext='.png'):
    """ 
    Save figure to specified location (create folder if it does not exist)
    """
    if not os.path.exists(folder):
        os.makedirs(folder)

    figname_full = os.path.join(folder, fname + ftype_ext)
    fig.savefig(figname_full, dpi=dpi, bbox_inches='tight')
    return 0


def debug_print(n, msg):
    """
    Debug printing for showing different level of debug information.
    """
    if n >= 0:
        tab_list = ['  '] * n
        dmsg = ''.join(tab_list) + 'DEBUG ' + msg
        print(dmsg)
    else:
        pass


def set_canvas_box(ax, bd_pts):
    """ 
    Set canvas of ax using map boundary pts.
    """
    xl, xh, yl, yh = bd_pts
    ax.set_xlim([xl, xh])
    ax.set_ylim([yl, yh])
    # ax.grid()
    ax.set_aspect('equal')
    return ax


def create_circle_patch(circle_array, color='tab:grey', alpha=0.8):
    """
    Given a list of same objects: (circles, ploygons, ...), create a
    patch collection object with certain color and trasparent alpha.
    Input: circle_array (num_pt * 3)
           each item [x, y, radius]
    """
    clist = []
    # prevent array size degeneration always turn it to 2D
    circle_array = np.reshape(circle_array, (-1, circle_array.shape[-1]))
    for item in circle_array:
        x, y, r = item
        circle = mp.Circle((x, y), r)
        clist.append(circle)
    patch_circles = PatchCollection(clist, color=color, alpha=alpha)
    return patch_circles


def create_arrowhead_patch():

    verts = [(0,0), (-1,-1), (2,0), (-1,1), (0,0)]
    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, 
                Path.LINETO, Path.CLOSEPOLY]
    return Path(verts, codes)

def set_peripheral(ax, *, t_str=None, x_str=None, y_str=None,
                   grid_on=True, plt_show=True, lg_on=False):
    """ Set plot peripherals
    """
    if t_str:
        ax.set_title(t_str, fontsize=18)
    if x_str:
        ax.set_xlabel(x_str, fontsize=15)
    if y_str:
        ax.set_ylabel(y_str, fontsize=15)
    if grid_on:
        ax.grid()
    if plt_show:
        plt.show()
    if lg_on:
        ax.legend()
    return ax

def create_arrow_patch(arrow_array, *,
                       sf=1, # scale factor
                       width=0.1, color='m'):
    """
    Given a list of same objects: (arrows), create a
    patch collection object with certain color and trasparent alpha.
    Input: arrow_array (num_pt * 4)
           each item [stx, sty, dx, dy]
    """
    arrow_list = []
    arrow_array = np.reshape(arrow_array,(-1,arrow_array.shape[-1]))
    for item in arrow_array:
        stx, sty, dx, dy = item
        dx*= sf
        dy*= sf
        this_arrow = mp.Arrow(stx, sty, dx, dy, width=width)
        arrow_list.append(this_arrow)
    patch_arrows = PatchCollection(arrow_list, color=color)
    return patch_arrows

def add_arrow(ax, arr_item, *, sf=1, width=0.1, color='m'):
    """ Add a arrow defined by arr_item (4,)
    arr_item [x, y, dx, dy]
    """
    stx, sty, dx, dy = arr_item
    arrow = mp.Arrow(stx, sty, sf*dx, sf*dy, color='m', width=0.1)
    ax.add_patch(arrow)
    return ax
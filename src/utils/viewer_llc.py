#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualization of double integrator system dynamics RGS log
Date : July-3 2020
Author: Zhichao Li @ UCSD
"""

import os
import pickle as pkl

import matplotlib
import numpy as np
from matplotlib import pyplot as plt

from utils.tools_plot_common import my_default_plt_setting

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

plt.ion()

class LogError(Exception):
    """ User Defined Exceptions for dIntSystem LogViewer.
    """
    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ''

    def __str__(self):
        if self.msg:
            return "LogViewer exception: {0}".format(self.msg)
        else:
            return "LogViewer exception"

class LogViewer:
    """
    Class for governor result visualization.
    """

    def __init__(self, log_fn=None):
        """
        Init the view using saved log file
        """
        if log_fn==None:
            pass
        else:
            self._filename = log_fn
            # unpack log file
            if os.path.isfile(self._filename):
                with open(self._filename, 'rb') as f:
                    res_log = pkl.load(f)
            else:
                err_msg = str(self._filename) +  ' not exists!'
                raise LogError(err_msg)

            # load default plot setting
            my_default_plt_setting()

            self.ss = res_log['controller']
            self.N = len(self.ss.log_zvec) # history length
            self._nu = self.ss._nu

            self._dt = self.ss.dt
            self.tvec = np.array(self.ss.log_t)

            # create local/global states for visualization
            self.xmat = np.array(self.ss.log_xvec)
            self.zmat = np.array(self.ss.log_zvec)
            self.umat = np.array(self.ss.log_uvec)    # ctrl

    def plot_state_local_frame(self, controller, angle_unit='rad', e_scale_factor=1):
        """ 
        plot states in local coordinates
        """
        fig, ax = plt.subplots()
        if controller=='Polar':
            eps = np.array([self.ss.eps_dist, self.ss.eps_angle])

            if angle_unit=='rad':
                labels=[r'$e$ (m)', r'$e_{\phi}$ (rad)', r'$d_{\phi}$ (rad)']
            else:
                labels=[r'$10 \times e$ (m)', r'$d_{\beta}$ (deg)', r'$d_{\phi}$ (deg)']
        elif controller=='Cone':
            eps = np.array([self.ss.eps_dist, 0.0])
            if angle_unit=='rad':
                labels=[r'$e$ (m)', r'$d_{\beta}$ (rad)', r'$z_{\phi}$ (rad)']
            else:
                labels=[r'$10 \times e$ (m)', r'$d_{\beta}$ (deg)', r'$z_{\phi}$ (deg)']
        
        from utils.tools_plot_dynsys import plot_polar_state
        fig, ax = plot_polar_state(fig, ax, self.tvec, self.xmat, eps=eps,
                                    labels=labels, controller=controller, angle_unit=angle_unit,
                                    e_scale_factor=e_scale_factor)


        return fig, ax

    def plot_ctrl(self, controller, ext_uvec=[], y_units=[], t_str=[]):
        """ 
        plot control signal
        """
        tvec = self.tvec[1:]
        t_str='Control Signals from '+controller+' Controller'
        fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)

        from utils.tools_plot_dynsys import plot_ctrl_2channel_bds

        if len(ext_uvec) > 0:
            print('Plot external uvec')
            ax1.plot(tvec[0:-1:10], ext_uvec[0:-1:10, 0], 'r--', label= r'$\tilde{u}_1$', lw=2)
            ax2.plot(tvec[0:-1:10], ext_uvec[0:-1:10, 1], 'r--', label= r'$\tilde{u}_2$', lw=2)
            ax1.legend(prop={'size':12}, loc='upper right')
            ax2.legend(prop={'size':12}, loc='upper right')

        if len(y_units) > 0:
            print('Add y label')
            ax1.set_ylabel(y_units[0], fontsize=15)
            ax2.set_ylabel(y_units[1], fontsize=15)

        fig, ax1, ax2 = plot_ctrl_2channel_bds(fig, 
                                               ax1, 
                                               ax2,
                                               tvec=tvec,
                                               umat=self.umat,
                                               ctrl_bds=[],
                                               t_str=t_str)

        plt.show()
        return fig, ax1, ax2

    def plot_pbf_stats(self):
        """ 
        plot status of PBF barrier functions (t) 
        """
        fig, ax1 = plt.subplots()
        ax1.plot(self.tvec, self.b_vec, label= r'$b$', lw=2)
        ax1.plot(self.tvec, 0 * self.tvec, 'k--', label= '0', lw=3)

        ax1.set_title('Parameterized Barrier Function Status', fontsize=18)
        ax1.legend(loc='upper right', prop={'size':12}) # change legend box loc
        plt.show()
        return fig, ax1

    def plot_rbt_trj_init(self, figure, map_size, *, ostar=[], goal_pt=[]):
        """ 
        plot states in cartesian coordinates for dIntSys
        """

        from utils.tools_plot_dynsys import plot_init_traj_unicycle
        trj_figure = plot_init_traj_unicycle(figure, 
                                             self.zmat,
                                             map_size=map_size,
                                             ostar=ostar,
                                             goal_pt=goal_pt)
        return trj_figure

    @staticmethod
    def pressQ_exit():
        """ 
        Press Keyboard `Q` to exit
        """
        while True:
            key_got = input('Press [Q] to quit, Press [R] to replay \n')
            if key_got == 'q' or key_got == 'Q':
                print('Received %c, Program terminated'  %key_got)
                break
            else:
                pass



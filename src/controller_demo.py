"""
Demo:
Goal tracking controller for Differential Drive wheeled robot with Unicycle Dynamics
    
    - controller_1 (Cone controller): goal position tracking
      [https://ieeexplore.ieee.org/document/388294?denied=]

    - controller_2 (Polar controller): goal pose (position & heading) alignment
      [https://arxiv.org/abs/2209.12648]

Author: Zhuolin Niu @ UCSD-ERL
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__))+"/../")

import pickle as pkl

import matplotlib as mpl
import numpy as np
from matplotlib import pyplot as plt

from utils.viewer_llc import LogViewer
from utils.tools_geometry import wf2bf2D, wrap_angle
from utils.tools_plot_common import my_default_plt_setting, save_fig_to_folder

from controller_polar import PolarController
from controller_cone import ConeController

my_default_plt_setting()

class ControllerDemo:

    def __init__(self,
                 zg,                          # goal pose [x, y, theta]
                 map_size,                    # sample intial pose in an [l(float)]-length square area
                 controller_type,             # position tracking [cone] or pose converging [polar] controller
                 control_bounds,              # control input constraints [u1min, u1max, u2min, u2max]
                 bi_direction=True,           # if True, active backward driving; if False, forward motion only
                 angle_resolution=5,          # angle resolution (degree) when sampling initial poses 
                 dt=0.05,                     # seconds/step
                 t_max=20.0,                  # max simulation time(s)
                 view_dynamics=False,         # plot states if True
                 save_fig=False,              # save figures as .pdf
                 full_trajectories=True,      # show trajectories from all initial poses to goal pose
                 random_initial_pose=False,   # if True, generate random initial poses 
                                              # if False, use pre-selected initial poses 
                 fixed_goal_pose=True,        # if True, use fixed goal pose; 
                                              # if False, goal heading point to individual initial position
                 ):
        
        self.zg = zg
        self.map_size = map_size
        self.angle_resolution = angle_resolution

        self.controller_type = controller_type
        self.bi_direction = bi_direction
        self.control_bounds = control_bounds

        self.dt = dt
        self.t_max = t_max
        self.tvec = np.arange(0.0, t_max+dt, dt)

        self.view_dynamics = view_dynamics
        self.save_fig = save_fig
        self.full_trajectories = full_trajectories
        self.random_initial_pose = random_initial_pose
        self.fixed_goal_pose = fixed_goal_pose

        pass

    def generate_random_pose(self, seg=False):
        """
        generate random pose [x,y,theta] with .1 resolution for x,y 
        and self.angle_resolution for theta in the (map_size x map_size) square area
        INPUT
        seg     [boolen] if True, position randomly on the first segment;
                         if False, position randomly on the whole area.
        """

        rslt_theta = self.angle_resolution
        map_size = self.map_size

        # get grid map range
        if seg:
            random_min = [0, 0]
        else:
            random_min = [-map_size*10, -map_size*10]
        random_max = [map_size*10+1, map_size*10+1]
        # generate random postion in grid cell coordinates
        position = 0.1*np.random.randint(random_min, random_max)
        # generate random heading angle (in degree)
        heading = np.random.randint(-180/rslt_theta, 180/rslt_theta+1)*rslt_theta                            
        init_pose = np.array([position[0], position[1], np.deg2rad(heading)])

        return init_pose

    def generate_initial_pose_list(self):
        """
        generate a list of initial poses
        """
        map_size = self.map_size
        if self.random_initial_pose:
            # segment map area to generate random positions
            seg_list = [[1,1],[-1,1],[-1,-1],[1,-1]]
            # random initial pose list container
            init_pose_list = np.empty((0,3))
            # get random initial poses
            for seg in seg_list:
                random_pose = self.generate_random_pose(seg=True)
                seg_array = np.array(seg)
                random_posotion = random_pose[:2]
                seg_position = seg_array*random_posotion
                random_pose[:2] = seg_position
                init_pose_list = np.vstack((init_pose_list, random_pose))
        else:
            # initial pose list container
            init_pose_list = np.zeros((8,3))

            # generate static initial position coords
            a = np.array([[-1, 0, 1]]).T
            p_all = np.hstack((np.repeat(a,3,axis=0),np.vstack((a,a,a))))
            p = np.delete(p_all, 4, axis=0)
            init_pose_list[:,:2] = map_size*p

            # generate static initial headings
            init_pose_list[0:2,2:] += np.pi/2
            init_pose_list[4:6,2:] += np.pi
            init_pose_list[2:4,2:] += -np.pi/2
        
        return init_pose_list

    def generate_param_dic(self):
        
        control_bounds = self.control_bounds
        controller_type = self.controller_type
        param_dic = {'control_bounds':control_bounds}
        
        if controller_type == 'Polar':
            ke = 0.6
            kdphi = 1.5 # heading goal
            kephi = 1.85 # heading error
            kv = 0.6 # speed
            control_gains={'kv':kv, 'kephi':kephi, 'kdphi':kdphi} 
            clf_params={'ke':ke, 'kdphi':kdphi}
            param_dic['control_gains']=control_gains
            param_dic['clf_params']=clf_params
        
        elif controller_type == 'Cone':
            kv, kw = 0.6, 1.85
            control_gains = {'kv':kv, 'kw':kw}
            param_dic['control_gains']=control_gains

        return param_dic

    def nonfixed_goal_heading(self, z0):
        """
        reset goal pose heading to point at initial position
        """
        zg = self.zg

        zg_bf = wf2bf2D(z0, zg[0:2])        # goal position in robot frame
        g_theta_raw = np.arctan(zg_bf[1]/zg_bf[0])+z0[2]
        zg[2] = wrap_angle(g_theta_raw)     # replace goal heading with non-fixed value

        self.zg = zg
        pass

    def init_controller(self, z):

        controller_type = self.controller_type
        zg = self.zg
        dt = self.dt
        param_dic = self.generate_param_dic()
        bi_direction = self.bi_direction

        if controller_type == 'Polar':
            unicycle_ss = PolarController(zg, 
                                          z,
                                          dt=dt,
                                          param_dic=param_dic,
                                          eps_dist=0.05,
                                          eps_angle=np.pi/20,
                                          bi_direction=bi_direction)

        elif controller_type == 'Cone':
            unicycle_ss = ConeController(zg, 
                                         z,
                                         param_dic=param_dic,
                                         dt=dt,
                                         eps_dist=0.001,
                                         bi_direction=bi_direction)

        return unicycle_ss

    @staticmethod
    def init_viewer():
        np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
        mpl.rcParams['pdf.fonttype'] = 42
        mpl.rcParams['ps.fonttype'] = 42
        plt.ion()
        pass

    def show_viewer(self, logname_full):

        controller = self.controller_type
        map_size = self.map_size
        zg = self.zg
        viewer = LogViewer(logname_full)

        if self.full_trajectories:
            full_trj_figure = viewer.plot_rbt_trj_init(self.figure, 
                                                       map_size=map_size,
                                                       goal_pt=zg)
            self.figure = full_trj_figure
        else:
            self.figure = plt.subplots()
            viewer.plot_rbt_trj_init(self.figure, 
                                     map_size=map_size,
                                     goal_pt=zg)
            if self.view_dynamics:
                viewer.plot_state_local_frame(controller=controller)
                viewer.plot_ctrl(controller=controller)

    def get_figure_name(self):
        """
        generate file name for saving figures
        """
        controller = self.controller_type
        if self.bi_direction:
            fig_name = controller+'BD_'
        else:
            fig_name = controller+'_'

        if self.fixed_goal_pose:
            fig_name  += 'GoalFixed_'
        else:
            fig_name  += 'GoalVary_'
        return fig_name

    def dump_data(self, ControllerObject):

        controller = self.controller_type

        folder = os.path.dirname(os.path.realpath(__file__))+"/../log"
        res_log = {'controller':ControllerObject}
        log_filename = controller+'_demo.pkl'
        if not os.path.exists(folder):
            os.makedirs(folder)

        logname_full = os.path.join(folder, log_filename)
        with open(logname_full, 'wb') as f:  # Python 3: open(..., 'wb')
            pkl.dump(res_log, f)

        return logname_full

    def demo_controller(self):

        zg = self.zg
        tvec = self.tvec
        t_max = self.t_max
        controller = self.controller_type

        self.init_viewer()
        save_fig = self.save_fig

        z0_list = self.generate_initial_pose_list()
        fig_name = self.get_figure_name()

        # initial figure to show all trajectories
        if self.full_trajectories:
            self.figure = plt.subplots(figsize=(10,10))

        for z0 in z0_list:
            print('z0  = %s' % z0)

            if not self.fixed_goal_pose:
                self.nonfixed_goal_heading(z0)

            # ----------- Initialize Controller ------------
            ControllerObject = self.init_controller(z0)
            print("---------- "+controller+" Controller Init Done--------------")

            # ----------- Main Loop ------------
            reached_goal_Flag = False  # goal configuration reached flag init as False
            while (not reached_goal_Flag) and tvec[ControllerObject.tidx] < t_max:
                ControllerObject.update(zg, debug_info=False)
                reached_goal_Flag = ControllerObject.check_goal_reached_rbt(debug_level=1)

            # ----------- Dump & View Data ----------
            logname_full = self.dump_data(ControllerObject)
            self.show_viewer(logname_full)

        plt.show(block=True)

        if save_fig:
            fig = self.figure
            save_fig_to_folder(fig, "fig", fig_name+'Trajectories', ftype_ext='pdf')


if __name__ == '__main__':

    # pick controller for demo [Cone] or [Polar]
    controller_type = "Cone"
    # pick map range [-map_size, map_size] 
    map_size = 1
    # set goal pose
    zg = np.array([0.0, 0.0, 0.0])
    # define control input constraints [u1min, u1max, u2min, u2max]
    control_bounds = np.array([-2, 2, -4, 4])

    RunDemo = ControllerDemo(zg=zg,
                             map_size=map_size, 
                             controller_type=controller_type,
                             control_bounds=control_bounds,
                             full_trajectories=True,
                             fixed_goal_pose=True,
                             bi_direction=True,
                             random_initial_pose=True,
                             view_dynamics=False,   # can be True only if (not full_trajectories)
                             )
    RunDemo.demo_controller()
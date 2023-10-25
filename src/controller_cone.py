#!/usr/bin/env python

# Cone controller for unicycle-like robot positional stabilization 
#   (angle convergence is not guranteed)
# https://arxiv.org/pdf/2209.12648.pdf

import numpy as np
import matplotlib as mpl
from matplotlib.path import Path
from scipy.integrate import solve_ivp

from utils.tools_dynsys import unicycle_carts
from utils.tools_geometry import wrap_angle, wf2bf2D


class ConeController:
    """ 
    Cone controller from Omur's technical report. Using this to compute 
    velocity control signal given current and desired robot states.
    """
    
    # Running status table, higher number better status
    NORMAL = 1
    GOAL_LOC_REACHED = 2

    def __init__(self, 
                 zg,                    # goal pose [g1, g2, (g_theta)] in world frame
                 z,                     # current robot pose in world frame [z1, z2, z_theta]
                 param_dic,             # parameter dictionary including control_bounds and control_gains
                 dt=0.05,               # seconds/step 
                 eps_dist=0.5,          # reached goal distance tolerance
                 bi_direction=False,    # if True, active bi_direction driving; if False, forward motion only
                 ):
        """
        Init cone controller
        System Dimensions:  
            states - global pose [z1, z2, z_theta]
            control input - local velocities [v, w]
            output - global trajectory [z1, z2, _],
        """
        print('--------Init Class ConeController Start------------')
        self.dt = dt
        self.eps_dist = eps_dist
        self.bi_direction = bi_direction
    
        self.gvec = zg
        self.zvec = z
        self.xvec = np.zeros(3)
        self.uvec = np.zeros(2)
        self.v = 0.0
        self.zg_bf = np.zeros(2)

        ctrl_params = param_dic['control_gains']
        self.ctrl_bounds = param_dic['control_bounds']

        # --------- log container-----------------
        self.log_uvec = np.empty((0,2))
        self.log_zvec = np.array([z])
        self.log_xvec = np.array([self.xvec])
        self.log_t = [0]

        print('zvec0 [z1, z2, z_theta] =  %s' % self.zvec)
        print('gvec0 [g1, g2, g_theta] =  %s' % self.gvec)
        print('xvec0 [e, d_phi, z_phi] =  %s' % self.xvec)

        if ctrl_params is not None:
            # controller design parameters
            self.kv = ctrl_params["kv"]
            self.kw = ctrl_params["kw"]
        else:
            print("[ConeController] use default params")
            self.kv = 0.5
            self.kw = 1.5

        self.tidx = 0
        self._nx = 3
        self._ng = len(zg)
        self._nu = 2
        self._ny = len(z)
        self.warning_msg = None
        self.status = ConeController.NORMAL
    
    def cmp_tracking_error(self):

        zg = self.gvec
        z = self.zvec
        zg_bf = wf2bf2D(z, zg[0:2])
        
        e = zg_bf[0]
        if self.bi_direction:
            if abs(zg_bf[1])<self.eps_dist:
                e_phi = 0.0
            else:
                e_phi = np.arctan(zg_bf[1]/zg_bf[0])
        else:
            e_phi = np.arctan2(zg_bf[1], zg_bf[0]) # tracking heading error

        if len(zg)==3:
            z_phi = wrap_angle(zg[2] - z[2]) # tracking orientation error
        else:
            z_phi = 0.0

        xvec = np.array([e, e_phi, z_phi])

        self.zg_bf = zg_bf
        self.xvec = xvec
        self.log_xvec = np.vstack((self.log_xvec, xvec))
        return xvec

    def generate_control(self, debug=False):
        """
        Generate velocity control signal (v, w) given current robot states and desired robot states.
        Input:
            @self.zvec: current robot states (z1, z2, z_theta)
            @self.zvec_g: desired robot states (g1, g2, g_theta)
        """

        xvec = self.cmp_tracking_error()
        zg_bf = self.zg_bf
        e, e_phi, _ = xvec

        # ------------------ speical case  ----------------
        if np.linalg.norm(zg_bf) < self.eps_dist:
            v = 0.0
            w = 0.0
            self.status = ConeController.GOAL_LOC_REACHED
            msg = "[cone controller]  status = GOAL_LOC_REACHED"
            print(msg)
                
            self.warning_msg = msg
            
        else:
            self.warning_msg = None
            self.status = ConeController.NORMAL
            # ------------------ normal case  ----------------
            if self.bi_direction:
                v = self.kv * e
            else:
                v = self.kv * max(0, e)
            w = self.kw * e_phi

        # check input constrains
        if v < self.ctrl_bounds[0]:
            v = self.ctrl_bounds[0]
        elif v > self.ctrl_bounds[1]:
            v = self.ctrl_bounds[1]
        if w < self.ctrl_bounds[2]:
            w = self.ctrl_bounds[2]
        elif w > self.ctrl_bounds[3]:
            w = self.ctrl_bounds[3]

        if debug:
            print("input z = [%.2f, %.2f, %.2f]" % (self.zvec[0], self.zvec[1], self.zvec[2]))
            print("input z_g = [%.2f, %.2f, %.2f]" % (self.gvec[0], self.gvec[1], self.gvec[2]))
            print("[e, e_phi] = [%.2f, %.2f]" % (e, e_phi))
            print("[v, w] = [%.2f, %.2f]" % (v, w))

        uvec = np.array([v, w])
        self.v = v
        self.w = w
        self.uvec = uvec
        self.log_uvec = np.vstack((self.log_uvec, uvec))

        return uvec
    
    def update(self, zg, debug_info=False):
        """ 
        update states of reference governor system via selected control
        law (pre-designed uvec_lyap, CLF-CBF-QP uvec_qp)
        This function is only for for unicyle now
        """
        self.gvec = zg
        zvec = self.zvec
        uvec = self.generate_control(debug=False)
        self.tidx +=1
        t = self.tidx * self.dt

        # real-robot state update
        zvec_sol = solve_ivp(unicycle_carts, [t-self.dt,t], zvec, t_eval=[t], args=(uvec[0], uvec[1]))
        zvec_up = zvec_sol.y[:,0]

        if debug_info:
            np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

            print("t = %4.2f" % (t))
            print('uvec = %s' % [uvec[0], uvec[1]])
            print('z  = %s' % zvec)
            print('z+ = %s' % zvec_up)
            print('--------------------------------------------')
            np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

        # update states and log
        self.zvec = zvec_up.copy()
        self.log_zvec = np.vstack((self.log_zvec, zvec_up))
        
        self.log_t.append(t)

        return zvec_up

    def check_goal_reached_rbt(self, debug_level=-1):
        """ check whether goal is reached for Unicycle
        """
        reached_flag = False
        
        if self.status == ConeController.GOAL_LOC_REACHED:
            if debug_level > 0:
                print("Goal configuraiton reached")
                reached_flag = True

        return reached_flag
    
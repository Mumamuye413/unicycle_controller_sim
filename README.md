
# unicycle_controller_sim
This repository features the code for two unicycle controllers, showcasing their performance in goal tracking. Each controller offers two distinct modes: the forward motion-only mode (as outlined in the reference paper) and the newly introduced bi-directional driving mode. It is part of Zhuolin Niu's [master thesis](https://escholarship.org/content/qt1jd778fm/qt1jd778fm.pdf) (Section 2.2.1 & Section 4.6).

### Goal Position Tracking Controller
#### Reference 

[Aykut İşleyen, Nathan van de Wouw, and Ömür Arslan. Feedback motion prediction for
safe unicycle robot navigation, 2023.](https://arxiv.org/pdf/2209.12648.pdf)

#### Method

This controller was derived from the robot's kinematic model and the motion model for tracking errors. It utilizes a proportional control gain applied to the error states to achieve precise goal position tracking.

* Notation & Goemetry Visualization

    Robot position $\quad \mathbf{z}_{\mathbf{p}} = [z_x, \ z_y]^T $

    Robot pose $\quad \mathbf{z} = [z_x, \ z_y, \ z_{\theta}]^T $

    Robot linear velocity $\quad v$

    Robot angular velocity $\quad \omega$

    Goal position in world frame $\{ W \}$ $\quad \mathbf{g}_{\mathbf{p}} = [g_x, \ g_y]^T$

    Goal pose in world frame $\{ W \}$ $\quad \mathbf{g} = [g_x, \ g_y, \ g_{\theta}]^T$

    Goal position in robot body frame $\{ B \}$ $\quad \mathbf{g}'=[g'_x, \ g'_y]^T$

    Tracking distance error $\quad e >0$

    Tracking heading error $\quad e_{\phi} \in [-\pi, \pi]$
  
    <img src="https://github.com/Mumamuye413/unicycle_controller_sim/blob/main/readme_fig/bdcone_geo.png" width="400">
    
* Kinematic Equations

```math
\begin{align}
    \dot{z}_x &= v \cos z_{\theta} \\
    \dot{z}_y &= v \sin z_{\theta} \\
    \dot{z}_{\theta} &= \omega
\end{align}
```

* Compute $\mathbf{g}'$

```math
    \mathbf{g}'= \mathbf{R}(-z_{\theta})(\mathbf{g} - \mathbf{z}_{\mathbf{p}}), 
    \quad \mathbf{R}(-z_{\theta}) = \left[ \begin{matrix} \quad \cos z_{\theta} \quad \sin z_{\theta} \\ \ -\sin z_{\theta} \quad \cos z_{\theta} \end{matrix} \right]
```

* Compute Error States

```math
    \begin{align}
    e'_{ \ \ } &= \max \ (0, \ g'_x) \\
    e_{\phi} &= \arctan2 \ (g'_y, \ g'_x)
    \end{align}
```

* Design Control Law

```math
\begin{align}
    v &= k_v \max(0, e') = k_v \max(0, \left[ \begin{matrix}  \cos z_{\theta} \\ \sin z_{\theta}  \end{matrix} \right] ^T (\mathbf{g}-\mathbf{z})), \quad k_v>0
    \\
    \omega &= k_{\omega} e_{\phi} = k_{\omega} \arctan2 ( \left[\begin{matrix} \ -\sin z_{\theta} \\ \cos z_{\theta} \end{matrix} \right] ^T (\mathbf{g}-\mathbf{z}),\quad \left[ \begin{matrix} \cos z_{\theta} \\ \sin z_{\theta} \end{matrix} \right] ^T (\mathbf{g}-\mathbf{z})), \quad k_{\omega}>0
\end{align}
```
    
* Simulated Robot Trajectories
<p align="center">
    <img src="/fig/Cone_GoalFixed_RandomInit_Trajectories.png" width="400">
    <img src="/fig/Cone_GoalFixed_SelectInit_Trajectories.png" width="400">
</p>


 Randomly sampled initial poses.      | Selected sampled initial poses.      
 -------------------------------------| -------------------------------------
 ![random trj](/fig/Cone_GoalFixed_RandomInit_Trajectories.png)| ![selected_trj](/fig/Cone_GoalFixed_SelectInit_Trajectories.png)

#### Modifidation for bi-directional motion

The bi-directional position tracking controller employs a notation consistent with the forward motion only position tracking controller introduced earlier, while incorporating adjustments in error state difination. The control law was subsequently formulated using the newly defined error states with the same equations.

* Updated Notations

    Bi-directional tracking distance error $\quad e \in \mathbb{R}$

    Bi-directional tracking heading error $\quad e_{\phi} \in [-\frac{\pi}{2}, \frac{\pi}{2}]$

* Goemetry Visualization

    <img src="https://github.com/Mumamuye413/unicycle_controller_sim/blob/main/readme_fig/bdcone_geo.png" width="400">

* Compute New Error States

```math
    \begin{align}
    e_{ \ \ } &= \ g'_x \\
    e_{\phi} &= \begin{cases}
    0, \quad \quad \quad \  g'_x=g'_y=0 \\
    \arctan \ (\frac{g'_y}{g'_x}), \quad  otherwise
    \end{cases}
    \end{align}
```

* Simulated Robot Trajectories

    * Randomly sampled initial poses
    <img src="/fig/ConeBD_GoalFixed_RandomInit_Trajectories.png" width="400">

    * Selected sampled initial poses
    <img src="/fig/ConeBD_GoalFixed_SelectInit_Trajectories.png" width="400">


### Goal Pose Alignment Controller
#### Reference 

[M Aicardi, G Casalino, A Biechi, and A Balestrino. Closed loop steering of unicycle-like
vehicles via lyapunov techniques.](http://www.centropiaggio.unipi.it/sites/default/files/vehicles-RA-MGZ95.pdf)

#### Method

The control law was developed by constructing a Lyapunov function based on error states, which are computed by projecting the robot's pose into the goal frame. The controller acts as a stabilizer, asymptotically driving all error states to zero, ensuring that the robot's pose aligns with the goal pose over time.

* Notation & Goemetry Visualization

    Robot position in world frame $\{ W \}$ $\quad \mathbf{z}_{\mathbf{p}} = [z_x, \ z_y]^T $

    Robot pose in world frame $\{ W \}$ $\quad \mathbf{z} = [z_x, \ z_y, \ z_{\theta}]^T $

    Robot linear velocity $\quad v$

    Robot angular velocity $\quad \omega$

    Goal position in world frame $\{ W \}$ $\quad \mathbf{g}_{\mathbf{p}} = [g_x, \ g_y]^T$

    Goal pose in world frame $\{ W \}$ $\quad \mathbf{g} = [g_x, \ g_y, \ g_{\theta}]^T$

    Robot position in goal frame $\{ G \}$ $\quad \mathbf{z}'_{\mathbf{p}} = [z'_x, \ z'_y]^T $

    Robot pose in goal frame $\{ G \}$ $\quad \mathbf{z}' = [z'_x, \ z'_y, \ z_{\phi}]^T $

    Tracking distance error $\quad e \in \mathbb{R}$

    Tracking heading error $\quad e_{\phi} \in [-\pi, \pi]$

    Heading alignment error $\quad e_{\delta} \in [-\pi, \pi]$

    <img src="https://github.com/Mumamuye413/unicycle_controller_sim/blob/main/readme_fig/polar_geo.png" width="400">

* Robot Kinematic Equations in frame $\{ G \}$

```math
    \begin{align}
    \dot{z}_x &= v \cos z_{\phi} \\
    \dot{z}_y &= v \sin z_{\phi} \\
    \dot{z}_{\phi} &= \omega
    \end{align}
```

* Compute Error States

```math
    \begin{align}
    \e &= \| \mathbf{z}'_{\mathbf{p}} \| \\
    \e_{\phi} &= \arctan2 \ (-z'_y, \ -z'_x) \\
    \e_{\delta} &= e_{\phi} - z_{\phi}
    \end{align}
```

* Error Dynamics Equations

```math
\begin{align}
\dot{e}_{\ \ } &= -v \cos e_{\delta} \\
\dot{e}_{\phi} &= v\frac{\sin e_{\delta}}{e} \\
\dot{e}_{\delta} &= v\frac{\sin e_{\delta}}{e} - \omega
\end{align}
```

* Lyapunov Functions

```math
    \begin{align}
        V &= V_1+V_2 = \frac{1}{2} \lambda e^2 + \frac{1}{2} (e^2_{\delta}+h e^2_{\phi});\ \lambda>0, h>0 \\

        \dot{V} &= \dot{V}_1 + \dot{V}_2 = -\lambda e v \cos e_{\delta} + e_{\delta} [ -\omega + v \frac{\sin e_{\delta}}{e_{\delta}} \frac{(e_{\delta} + h e_{\phi})} {e}]
    \end{align}
```

* Control Law Design

```math
\begin{align}
\dot{V}_1 \leq 0: \quad  &v = k_v e \cos e_{\delta}; \\

\dot{V}_2 \leq 0: \quad &\omega = k_{\delta} e_{\delta} + k_v \frac{\cos e_{\delta} \sin e_{\delta}}{e_{\delta}} (e_{\delta} + h e_{\phi})
```

* Simulated Robot Trajectories

#### Modifidation for bi-directional motion

To enable full backward motion for the robot when the goal position falls within the robot's rear half-plane, we introduce a virtual goal pose that located at the same position but points in the opposite direction of the actual goal pose. The robot then aligns its rear heading with this virtual goal's heading to maintain alignment with the actual goal heading.

* New & Updated Notations

    Virtual goal pose in world frame $\{ W \}$ $\quad \hat{\mathbf{g}} = [g_x, \ g_y, \ \hat{g}_{\theta}]^T$

    Robot pose in virtual goal frame $\{ \hat{G} \}$ $\quad \hat{\mathbf{z}}' = [\hat{z}'_x, \ \hat{z}'_y, \ \hat{z}_{\phi}]^T $

    Tracking heading error w.r.t. $\hat{g}_{\theta}$ $\quad \hat{e}_{\phi} \in [-\frac{\pi}{2}, \frac{\pi}{2}]$

    Heading alignment error w.r.t. $\hat{g}_{\theta}$ $\quad \hat{e}_{\delta} \in [-\frac{\pi}{2}, \frac{\pi}{2}]$

* Goemetry Visualization

    <img src="https://github.com/Mumamuye413/unicycle_controller_sim/blob/main/readme_fig/bdpolar_geo.png" width="400">

* Compute Error States for Backwards Tracking

```math
    \begin{align}
    \e &= \| \mathbf{z}'_{\mathbf{p}} \| \\
    \hat{e}_{\phi} &= \arctan \fraq{z'_y}{z'_x} \\
    \hat{e}_{\delta} &= \hat{e}_{\phi} - z_{\phi}
    \end{align}
```

* Error Dynamics Equations for Backwards Tracking

```math
    \begin{align}
    \dot{e}_{\ \ } &= -v \cos e_{\delta} \\
    \dot{\hat{e}}_{\phi} &= v\frac{\sin \hat{e}_{\delta}}{e} \\
    \dot{\hat{e}}_{\delta} &= v\frac{\sin \hat{e}_{\delta}}{e} - \omega
    \end{align}
```

* Integrated Control Law for Bi-directional Tracking

```math
    \begin{aligned}
    v &= \begin{cases} k_v e \cos \hat{e}_{\delta}, \quad &[\cos z_{\theta}, \sin z_{\theta} ](\mathbf{g}_{\mathbf{p}}-\mathbf{z}_\{\mathbf{p}})< 0 \\
        k_v e \cos e_{\delta}, \quad &otherwise
        \end{cases} \\
    \omega &= \begin{cases} k_{\delta} \hat{e}_{\delta} + k_v \frac{\cos \hat{e}_{\delta} \sin \hat{e}_{\delta}}{\hat{e}_{\delta}} (\hat{e}_{\delta}+h \hat{e}_{\phi}), 
    \quad &[\cos z_{\theta}, \sin z_{\theta} ](\mathbf{g}_\mathbf{p}-\mathbf{z}_{\mathbf{p}})< 0 \\
    k_{\delta} e_{\delta} + k_v \frac{\cos e_{\delta} \sin e_{\delta}}{e_{\delta}} (e_{\delta}+h e_{\phi}),  \quad &otherwise
            \end{cases}
    \end{aligned}
```

* Simulated Robot Trajectories









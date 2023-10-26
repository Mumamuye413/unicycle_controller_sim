# unicycle_controller_sim
This repository features the code for two unicycle controllers, showcasing their performance in goal tracking. Each controller offers two distinct modes: the forward motion-only mode (as outlined in the reference paper) and the newly introduced bi-directional driving mode. It is part of Zhuolin Niu's [master thesis](https://escholarship.org/content/qt1jd778fm/qt1jd778fm.pdf).

### Goal Position Tracking Controller
#### Reference 

[Aykut İşleyen, Nathan van de Wouw, and Ömür Arslan. Feedback motion prediction for
safe unicycle robot navigation, 2023.](https://arxiv.org/pdf/2209.12648.pdf)

#### Method

This controller was designed from the robot's kinematic model and the tracking errors' motion model. 

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
    
* Kinematic Equations in Cartesian coordinates system

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
    * Randomly sampled initial poses
    <img src="/fig/Cone_GoalFixed_RandomInit_Trajectories.png" width="400">

    * Selected sampled initial poses
    <img src="/fig/Cone_GoalFixed_SelectInit_Trajectories.png" width="400">

#### Modifidation for bi-directional motion

The bi-directional position tracking controller employs a notation consistent with the forward motion-only position tracking controller introduced earlier, while incorporating adjustments in error state difination. The control law was subsequently formulated using the newly defined error states with the same equations.

* New Notation

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











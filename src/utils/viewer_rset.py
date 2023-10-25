import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.path import Path
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from shapely.plotting import plot_polygon

def generate_cone_set(points, radius):
    """
    input: points - [tangent_point1, tangent_point2, p, pg] - a list of 2d points coords
                    list elements - (1d array [x, y]) 
           radius - ice-cream ball radius 
    
    Output: iccone_set - shapely.geometry object
    """
    tpt1wf, tpt2wf, p, p_g = points

    ic_ball = Point(p_g[0], p_g[1]).buffer(radius)
    cone_vrts = [(p[0],p[1]), (tpt1wf[0],tpt1wf[1]), (tpt2wf[0],tpt2wf[1])]
    ic_cone = Polygon(cone_vrts)
    # ---------- special case ------------
    # other halp plane: Euclidean ballx
    # too close to goal: point
    if np.all(tpt1wf==p):
        ic_ball = Point(p_g[0], p_g[1]).buffer(0.01)
        cone_set = ic_ball

    # heading aline with tracking: line
    elif radius==0 and np.linalg.norm(tpt1wf-p)>0:
        ic_cone = LineString([(p[0],p[1]), (p_g[0],p_g[1])]).buffer(0.01)
        cone_set = ic_cone

    # ---------- normal case ------------
    else:
        cone_set = unary_union([ic_ball, ic_cone])

    return cone_set

def plot_cone_set(points, radius, ax):
    """
    Input: cone_set - shapely.geometry object
    Output: ax - plt.axes
    """
    cone_set = generate_cone_set(points, radius)

    polyplt = plot_polygon(cone_set, ax=ax, add_points=False, 
                           facecolor='lightblue', alpha=0.5, edgecolor='darkblue')
    ax = polyplt.axes
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    return ax




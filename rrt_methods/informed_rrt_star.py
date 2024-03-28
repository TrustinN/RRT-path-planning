import numpy as np
from .rrt_utils import Ellipse
from . import rrt_star
from . import rrt_connect


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(start, end, map, step_size, max_iter):

    # Find a path first
    path = rrt_connect.rrt_run(start, end, map, step_size, max_iter, clear=True)
    d_worst = 0
    for i in range(len(path) - 2):
        p = path[i + 1]
        curr_dist = np.linalg.norm(p - start) + np.linalg.norm(p - end)
        if curr_dist > d_worst:
            d_worst = curr_dist

    # start informed rrt sampling
    ellipse_scope = Ellipse(start, end, d_worst)
    map.add_path([start, end])
    map.sample_init("ellipse", ellipse_scope)

    # sample number of points proportional to area of ellipse
    scale = d_worst / np.linalg.norm(start - end)
    max_iter = max_iter * scale / 1.5

    return rrt_star.rrt_run(start, end, map, step_size, max_iter)










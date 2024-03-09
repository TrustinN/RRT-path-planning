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
    path_length = 0
    for i in range(len(path) - 1):
        path_length += np.linalg.norm(path[i] - path[i + 1])

    # start informed rrt sampling
    ellipse_scope = Ellipse(start, end, path_length)
    map.add_path([start, end])
    map.sample_init("ellipse", ellipse_scope)

    return rrt_star.rrt_run(start, end, map, step_size, max_iter)










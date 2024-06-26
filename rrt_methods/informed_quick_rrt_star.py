import numpy as np
from utils.maps.map_utils import SampleScope
from . import quick_rrt_star
from . import rrt_star_connect


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter):

    # Find a path first
    start, end = map.path[0], map.path[1]
    path, _, _ = rrt_star_connect.rrt_run(map, step_size, max_iter, clear=True)
    d_worst = 0

    for i in range(len(path) - 2):
        p = path[i + 1]
        curr_dist = np.linalg.norm(p - start) + np.linalg.norm(p - end)
        if curr_dist > d_worst:
            d_worst = curr_dist

    # start informed rrt sampling
    if map.dim == 2:
        ellipse_scope = SampleScope.Ellipse(end, start, d_worst)

    elif map.dim == 3:
        ellipse_scope = SampleScope.Spheroid(end, start, d_worst)

    map.add_path([start, end])
    map.sample_init(ellipse_scope)

    # sample number of points proportional to area of ellipse
    scale = d_worst / np.linalg.norm(start - end)
    max_iter = max_iter * scale / 1.5

    path, graph, _ = quick_rrt_star.rrt_run(map, step_size, max_iter)

    return path, graph, _




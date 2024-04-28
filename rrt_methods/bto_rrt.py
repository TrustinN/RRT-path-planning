import math
import numpy as np
from .rrt_utils import vertex
from .rrt_utils import graph_init
from .rrt_utils import rrt_step
from .rrt_utils import down_sample, up_sample, KPSOptimization
from utils.rtree.rtree_utils import IndexRecord


###############################################################################
# RRT Method                                                                  #
###############################################################################

def bto_extend(graph, map, flag, step_size):
    x_rand = flag.value
    while True:

        p_rand = IndexRecord(None, x_rand)
        v_near = graph.NearestNeighbor(p_rand)[0]
        x_new = rrt_step(x_rand, v_near, step_size)

        if not map.intersects_line([x_new, v_near.value]):

            v_new = vertex(value=x_new,
                           parent=None,
                           dist_to_root=math.inf)
            v_near.add_neighbor(v_new)
            graph.add_vertex(v_new)
            if np.linalg.norm(v_near.value - flag.value) < step_size:
                return v_new, True

            return v_new, False

        while True:
            x_rand = map.sample()

            if map.in_free_space(x_rand):
                break


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter):

    v_start, v_end, t_a, t_b = graph_init(map=map)
    # We want t_a to extend towards v_end
    # We want t_b to extend towards the most recent node of t_a
    a_flag = v_start
    b_flag = v_end

    iter = 0
    v_new = None
    path = []
    while iter < max_iter:
        iter += 1

        a_flag, found = bto_extend(t_a, map, b_flag, step_size)

        if found:
            path = t_a.backtrack_root(v_new)
            break

        v_new, found = bto_extend(t_b, map, a_flag, step_size)

        if found:
            near = IndexRecord(None, v_new.value)
            path = t_b.backtrack_root(v_new) + t_a.backtrack_root(t_a.NearestNeighbor(near)[0])[::-1]
            path = path[::-1]
            break

    path = down_sample(map, path)
    path = up_sample(map, path, 2 * iter)
    path = down_sample(map, path)
    path = KPSOptimization(path, map, num=20)

    return path, t_a, t_b




























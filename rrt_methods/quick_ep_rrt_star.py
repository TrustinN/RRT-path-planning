import math
from utils.maps.map_utils import SampleScope
from . import rrt_star_connect
from .rrt_utils import vertex
from .rrt_utils import graph_init
from .rrt_utils import rrt_step
from .rrt_utils import rrt_q_rewire
from utils.rtree.rtree_utils import IndexRecord


###############################################################################
# RRT Method                                                                  #
###############################################################################

def set_ep(map, path):
    # start informed rrt sampling
    if map.dim == 2:
        exp_scope = SampleScope.EpRegion(path, 20)

    elif map.dim == 3:
        exp_scope = SampleScope.EpRegion(path, 20)

    map.sample_init(exp_scope)


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter):

    # Find a path first
    path, graph, _ = rrt_star_connect.rrt_run(map, step_size, max_iter, clear=True)
    set_ep(map, path)

    # sample number of points proportional to area of ellipse
    # scale = d_worst / np.linalg.norm(start - end)

    v_start, v_end, graph, _ = graph_init(map=map)
    prev_best = v_end.dist_to_root

    iter = 0
    while iter < max_iter / 2:

        if v_end.dist_to_root < prev_best:
            if v_end.dist_to_root < prev_best - step_size / 5:

                prev_best = v_end.dist_to_root
                path = graph.backtrack(v_start, v_end)
                set_ep(map, path)
                v_start, v_end, graph, _ = graph_init(map=map)

            else:
                break

        p_rand = map.sample()
        p_test = IndexRecord(None, p_rand)
        v_near = graph.NearestNeighbor(p_test)[0]
        p_new = rrt_step(p_rand, v_near, step_size)

        if map.in_free_space(p_new):
            iter += 1
            v_new = vertex(value=p_new,
                           parent=None,
                           dist_to_root=math.inf
                           )
            rrt_q_rewire(v_new, graph, map, 1.2, 2, step_size, v_end)

    return path, graph, _



















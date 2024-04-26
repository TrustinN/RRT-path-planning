import math
from .rrt_utils import vertex
from .rrt_utils import graph_init
from .rrt_utils import rrt_step
from .rrt_utils import rrt_rewire
from utils.rtree.rtree_utils import IndexRecord


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter):

    v_start, v_end, graph, _ = graph_init(map=map)

    iter = 0
    while iter < max_iter:

        p_rand = map.sample()
        p_test = IndexRecord(None, p_rand)
        v_near = graph.NearestNeighbor(p_test)[0]
        p_new = rrt_step(p_rand, v_near, step_size)

        if map.in_free_space(p_new):
            iter += 1
            v_new = vertex(value=p_new,
                           parent=None,
                           dist_to_root=math.inf,
                           )
            rrt_rewire(v_new, graph, map, 3, step_size, v_end)

    if v_end.dist_to_root == math.inf:
        path = []
    else:
        path = graph.backtrack(v_start, v_end)

    return path, graph, None















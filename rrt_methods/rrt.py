import math
from .rrt_utils import graph_init
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
    while not v_end.parent and iter < max_iter:
        iter += 1

        p_rand = map.sample()

        p_test = IndexRecord(None, p_rand)
        v_near = graph.NearestNeighbor(p_test)[0]
        p_near = v_near.value

        if map.in_free_space(p_rand):
            if not map.intersects_line([p_rand, p_near]):
                v_new = graph.make_vertex(value=p_rand,
                                          parent=None,
                                          )
                v_near.add_neighbor(v_new)
                graph.add_vertex(v_new)
                iter += 1
                if not map.intersects_line([v_end.value, v_new.value]):
                    if v_new.dist_to_root + v_new.dist_to(v_end) < v_end.dist_to_root:
                        v_new.add_neighbor(v_end)

    if v_end.dist_to_root == math.inf:
        path = []

    else:
        path = graph.backtrack(v_start, v_end)

    return path, graph, None









from .rrt_utils import graph_init
from .rrt_utils import rrt_extend_connect
from .rrt_utils import rrt_connect_path
from utils.rtree.rtree_utils import IndexRecord


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter, clear=False):

    v_start, v_end, t_start, t_end = graph_init(map=map)

    connect = False
    c1, c2 = None, None

    iter = 0
    while iter < max_iter:
        iter += 1

        p_rand = map.sample()
        p_test = IndexRecord(None, p_rand)
        v_near = t_start.NearestNeighbor(p_test)[0]
        p_near = v_near.value

        if map.in_free_space(p_rand):
            connect, c1, c2 = rrt_extend_connect(p_rand,
                                                 p_near,
                                                 v_near,
                                                 map,
                                                 step_size,
                                                 t_start, t_end,
                                                 )

        if connect:
            break

        p_test = IndexRecord(None, p_rand)
        v_near = t_end.NearestNeighbor(p_test)[0]
        p_near = v_near.value

        if map.in_free_space(p_rand):
            connect, c1, c2 = rrt_extend_connect(p_rand,
                                                 p_near,
                                                 v_near,
                                                 map,
                                                 step_size,
                                                 t_end, t_start,
                                                 )

        if connect:
            break

    if clear:
        t_start.clear()
        t_end.clear()

    if not connect:
        path = []
    else:
        path = rrt_connect_path(v_start, v_end, t_start, t_end, c1, c2)

    return path, t_start, t_end



























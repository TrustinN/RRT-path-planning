from .rrt_utils import graph_init
from .rrt_utils import closest_point
from .rrt_utils import Sampler
from .rrt_utils import in_free_space
from .rrt_utils import rrt_extend_connect
from .rrt_utils import rrt_connect_path


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(start, end, map, step_size, max_iter, clear=False):

    sampler = Sampler(map)
    v_start, v_end, t_start, t_end = graph_init(start, end)

    connect = False
    c1, c2 = None, None

    iter = 0
    while iter < max_iter:
        iter += 1

        p_rand = sampler.sample()
        v_near = closest_point(p_rand, t_start.vertices)
        p_near = v_near.value

        if in_free_space(p_rand, map.region, map.obstacles):
            connect, c1, c2 = rrt_extend_connect(p_rand,
                                                 p_near,
                                                 v_near,
                                                 map.region,
                                                 map.obstacles,
                                                 step_size,
                                                 t_start, t_end,
                                                 )

        if connect:
            break

        v_near = closest_point(p_rand, t_end.vertices)
        p_near = v_near.value

        if in_free_space(p_rand, map.region, map.obstacles):
            connect, c1, c2 = rrt_extend_connect(p_rand,
                                                 p_near,
                                                 v_near,
                                                 map.region,
                                                 map.obstacles,
                                                 step_size,
                                                 t_end, t_start,
                                                 )

        if connect:
            break

    if clear:
        t_start.clear()
        t_end.clear()

    if iter == max_iter:
        return []
    else:
        return rrt_connect_path(v_start, v_end, t_start, t_end, c1, c2)









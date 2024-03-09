import math
from .rrt_utils import vertex
from .rrt_utils import graph_init
from .rrt_utils import Sampler
from .rrt_utils import closest_point
from .rrt_utils import in_free_space
from .rrt_utils import rrt_extend_connect
from .rrt_utils import rrt_connect_path
from .rrt_utils import ep_rrt_sample
from .rrt_utils import rrt_rewire


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(start, end, map, step_size, max_iter):

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

    path = rrt_connect_path(v_start, v_end, t_start, t_end, c1, c2)
    t_start.clear()
    t_end.clear()

    new_iter = 0
    v_start, v_end, t_start, _ = graph_init(start, end, connect=False)

    while new_iter < max_iter / 10:
        p_new = ep_rrt_sample(step_size, path)
        if in_free_space(p_new, map.region, map.obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           position=0,
                           parent=None,
                           plots=[],
                           )
            rrt_rewire(v_new, t_start, map.region, map.obstacles, 5,  step_size, v_end)
        new_iter += 1

    if v_end.dist_to_root == math.inf:
        return []
    else:
        return t_start.backtrack(v_start, v_end)







from .rrt_utils import vertex
from .rrt_utils import graph_init
from .rrt_utils import Sampler
from .rrt_utils import closest_point
from .rrt_utils import rrt_step
from .rrt_utils import in_free_space
from .rrt_utils import rrt_rewire
from .rrt_utils import rrt_connect
from .rrt_utils import rrt_connect_path


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(start, end, map, step_size, max_iter):

    sampler = Sampler(map)
    v_start, v_end, t_start, t_end = graph_init(start, end, connect=True)
    c1, c2 = None, None

    iter = 0
    while iter < max_iter:
        iter += 1

        p_rand = sampler.sample()
        v_near = closest_point(p_rand, t_start.vertices)
        p_new = rrt_step(p_rand, v_near, step_size)
        if in_free_space(p_new, map.region, map.obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           position=0,
                           parent=None,
                           plots=[],
                           )
            if rrt_rewire(v_new, t_start, map.region, map.obstacles, 5, step_size, v_end, connect=True):
                c1 = v_new.parent
                connect = rrt_connect(v_new, t_start, t_end, map.region, map.obstacles, 5, step_size)
                if connect:
                    c2 = v_new
                    break

        v_near = closest_point(p_rand, t_end.vertices)
        p_new = rrt_step(p_rand, v_near, step_size)
        if in_free_space(p_new, map.region, map.obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           position=0,
                           parent=None,
                           plots=[],
                           )
            if rrt_rewire(v_new, t_end, map.region, map.obstacles, 5, step_size, v_end, connect=True):
                c2 = v_new.parent
                connect = rrt_connect(v_new, t_end, t_start, map.region, map.obstacles, 5, step_size)
                if connect:
                    c1 = v_new
                    break

    if iter == max_iter:
        return []
    else:
        return rrt_connect_path(v_start, v_end, t_start, t_end, c1, c2)








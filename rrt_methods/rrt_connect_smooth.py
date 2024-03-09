from .rrt_utils import graph_init
from .rrt_utils import closest_point
from .rrt_utils import sample_free, find_bounding_box
from .rrt_utils import in_free_space
from .rrt_utils import rrt_extend_connect
from .rrt_utils import rrt_connect_path


###############################################################################
# RRT Method                                                                  #
###############################################################################


def rrt_smoothing(path):
    
    return


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(start, end, region, obstacles, step_size, max_iter):
    min_x, max_x, max_y, min_y = find_bounding_box(region)
    v_start, v_end, t_start, t_end = graph_init(start, end)

    connect = False
    c1, c2 = None, None

    iter = 0
    while iter < max_iter:
        iter += 1

        p_rand = sample_free(min_x, max_x, max_y, min_y, region, obstacles)
        v_near = closest_point(p_rand, t_start.vertices)
        p_near = v_near.value

        if in_free_space(p_rand, region, obstacles):
            connect, c1, c2 = rrt_extend_connect(p_rand,
                                                 p_near,
                                                 v_near,
                                                 region,
                                                 obstacles,
                                                 step_size,
                                                 t_start, t_end,
                                                 )

        if connect:
            break

        v_near = closest_point(p_rand, t_end.vertices)
        p_near = v_near.value

        if in_free_space(p_rand, region, obstacles):
            connect, c1, c2 = rrt_extend_connect(p_rand,
                                                 p_near,
                                                 v_near,
                                                 region, obstacles,
                                                 step_size,
                                                 t_end, t_start,
                                                 )

        if connect:
            break

    if iter == max_iter:
        path = []
    else:
        path = rrt_connect_path(v_start, v_end, t_start, t_end, c1, c2)

    








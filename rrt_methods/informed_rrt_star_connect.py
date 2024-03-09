import math
from .rrt_utils import vertex
from .rrt_utils import graph_init
from .rrt_utils import sample_free, find_bounding_box
from .rrt_utils import rrt_step
from .rrt_utils import rrt_connect
from .rrt_utils import rrt_rewire
from .rrt_utils import in_free_space
from .rrt_utils import Ellipse
from .rrt_utils import ellipse_sample
from .rrt_utils import rrt_connect_path


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(start, end, region, obstacles, step_size, max_iter):
    min_x, max_x, max_y, min_y = find_bounding_box(region)
    v_start, v_end, t_start, t_end = graph_init(start, end, connect=True)
    c1, c2 = None, None

    iter = 0
    while iter < max_iter:
        iter += 1

        p_rand = sample_free(min_x, max_x, max_y, min_y, region, obstacles)

        p_new = rrt_step(p_rand, t_start, step_size)
        if in_free_space(p_new, region, obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           position=0,
                           parent=None,
                           plots=[],
                           )
            if rrt_rewire(v_new, t_start, region, obstacles, 5, step_size, v_end, connect=True):
                c1 = v_new.parent
                connect = rrt_connect(v_new, t_start, t_end, region, obstacles, 5, step_size)
                if connect:
                    c2 = v_new
                    break

        p_new = rrt_step(p_rand, t_end, step_size)
        if in_free_space(p_new, region, obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           position=0,
                           parent=None,
                           plots=[],
                           )
            if rrt_rewire(v_new, t_end, region, obstacles, 5, step_size, v_end, connect=True):
                c2 = v_new.parent
                connect = rrt_connect(v_new, t_end, t_start, region, obstacles, 5, step_size)
                if connect:
                    c1 = v_new
                    break

    t_start.clear()
    t_end.clear()

    # start informed rrt sampling
    path_length = c1.dist_to_root + c2.dist_to_root + c1.dist_to(c2)

    iter = 0
    el = Ellipse(start, end, path_length)
    v_start, v_end, t_start, t_end = graph_init(start, end, connect=True)
    c1, c2 = None, None
    min_dist = math.inf

    while iter < max_iter / 4:
        iter += 1

        p_rand = ellipse_sample(el, buffer=1.1)
        p_new = rrt_step(p_rand, t_start, step_size)

        if in_free_space(p_new, region, obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           position=0,
                           parent=None,
                           plots=[],
                           )
            if rrt_rewire(v_new, t_start, region, obstacles, 5, step_size, v_end, connect=True):
                c1 = v_new.parent
                connect = rrt_connect(v_new, t_start, t_end, region, obstacles, 5, step_size)
                curr_dist = v_new.dist_to_root + c1.dist_to_root + v_new.dist_to(c1)
                if connect and (curr_dist < min_dist or not c2):
                    c2 = v_new
                    min_dist = curr_dist

        p_new = rrt_step(p_rand, t_end, step_size)
        if in_free_space(p_new, region, obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           position=0,
                           parent=None,
                           plots=[],
                           )
            if rrt_rewire(v_new, t_end, region, obstacles, 5, step_size, v_end, connect=True):
                c2 = v_new.parent
                connect = rrt_connect(v_new, t_end, t_start, region, obstacles, 5, step_size)
                curr_dist = v_new.dist_to_root + c2.dist_to_root + v_new.dist_to(c2)
                if connect and (curr_dist < min_dist or not c1):
                    c1 = v_new
                    min_dist = curr_dist

    return rrt_connect_path(v_start, v_end, t_start, t_end, c1, c2)















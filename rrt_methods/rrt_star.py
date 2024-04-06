import math
from .rrt_utils import graph_init
from .rrt_utils import rrt_step
from .rrt_utils import in_free_space
from .rrt_utils import rrt_rewire
from r_trees.r_tree_utils import IndexRecord
from utils.map_utils import plot_path


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter, plotting=False):

    v_start, v_end, graph, _ = graph_init(map=map, plotting=plotting)

    iter = 0
    while iter < max_iter:
        iter += 1

        p_rand = map.sample()
        p_test = IndexRecord(None, p_rand)
        v_near = graph.NearestNeighbor(p_test)
        p_new = rrt_step(p_rand, v_near, step_size)

        if in_free_space(p_new, map.region, map.obstacles):
            v_new = graph.make_vertex(value=p_new,
                                      neighbors=[],
                                      position=0,
                                      parent=None,
                                      )
            rrt_rewire(v_new, graph, map.region, map.obstacles, 5, step_size, v_end)

    if v_end.dist_to_root == math.inf:
        path = []
    else:
        path = graph.backtrack(v_start, v_end)

    if plotting:
        plot_path(path, c="#000000", ax=map.ax)

    return path




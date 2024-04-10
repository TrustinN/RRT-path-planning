import math
from .rrt_utils import intersects_objects
from .rrt_utils import graph_init
from .rrt_utils import rrt_step
from .rrt_utils import in_free_space
from .rrt_utils import rrt_q_rewire
from r_trees.r_tree_utils import IndexRecord

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

        p_rand = map.sample()
        p_test = IndexRecord(None, p_rand)
        v_near = graph.NearestNeighbor(p_test)
        p_new = rrt_step(p_rand, v_near, step_size)

        if in_free_space(p_new, map.region, map.obstacles):
            iter += 1
            v_new = graph.make_vertex(value=p_new,
                                      neighbors=[],
                                      parent=None,
                                      dist_to_root=math.inf,
                                      )
            if v_start.equals(v_near):
                if not intersects_objects(map, v_start.value, v_new.value):
                    v_start.add_neighbor(v_new)
                    graph.add_vertex(v_new)
            else:
                rrt_q_rewire(v_new, graph, map, 1.2, 2, step_size, v_end)

    if v_end.dist_to_root == math.inf:
        path = []
    else:
        path = graph.backtrack(v_start, v_end)

    if plotting:
        map.plot_path(path)

    return path







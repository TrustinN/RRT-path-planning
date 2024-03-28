import math
import numpy as np
import matplotlib.pyplot as plt
from .rrt_utils import vertex
from .rrt_utils import intersects_objects
from .rrt_utils import graph_init
from .rrt_utils import Sampler
from .rrt_utils import rrt_step
from .rrt_utils import in_free_space
from .rrt_utils import rrt_q_rewire
from r_trees.r_tree_utils import IndexRecord


###############################################################################
# RRT Method                                                                  #
###############################################################################


np.random.seed(450)


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(start, end, map, step_size, max_iter):

    sampler = Sampler(map)
    v_start, v_end, graph, _ = graph_init(start, end)

    iter = 0
    while iter < max_iter:
        iter += 1

        p_rand = sampler.sample()
        if sampler.scope == "ellipse":
            plt.scatter(p_rand[0], p_rand[1])
        p_test = IndexRecord(None, p_rand)
        v_near = graph.NearestNeighbor(p_test)
        p_new = rrt_step(p_rand, v_near, step_size)

        if in_free_space(p_new, map.region, map.obstacles):
            v_new = vertex(value=p_new,
                           neighbors=[],
                           parent=None,
                           plots=[],
                           dist_to_root=math.inf,
                           )
            if v_start.equals(v_near):
                if not intersects_objects(map.region, map.obstacles, v_start.value, v_new.value):
                    v_start.add_neighbor(v_new)
                    graph.add_vertex(v_new)
            else:
                rrt_q_rewire(v_new, graph, map.region, map.obstacles, 1.2, 2, step_size, v_end)

    if v_end.dist_to_root == math.inf:
        return []
    else:
        return graph.backtrack(v_start, v_end)









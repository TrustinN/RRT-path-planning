from .rrt_utils import graph_init
from .rrt_utils import Sampler
from .rrt_utils import intersects_objects
from .rrt_utils import in_free_space
from r_trees.r_tree_utils import IndexRecord
from utils.map_utils import plot_path


###############################################################################
# RRT Method                                                                  #
###############################################################################


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter, plotting=False):

    sampler = Sampler(map)
    v_start, v_end, graph, _ = graph_init(map=map, plotting=plotting)

    iter = 0
    while v_end.num_neighbors < 1 and iter < max_iter:
        iter += 1

        p_rand = sampler.sample()

        p_test = IndexRecord(None, p_rand)
        v_near = graph.NearestNeighbor(p_test)
        p_near = v_near.value

        if in_free_space(p_rand, map.region, map.obstacles):
            if not intersects_objects(map.region, map.obstacles, p_rand, p_near):
                v_new = graph.make_vertex(value=p_rand,
                                          neighbors=[],
                                          position=0,
                                          parent=None,
                                          )
                v_near.add_neighbor(v_new)
                graph.add_vertex(v_new)
                iter += 1
                if not intersects_objects(map.region, map.obstacles, v_end.value, v_new.value):
                    if v_new.dist_to_root + v_new.dist_to(v_end) < v_end.dist_to_root:
                        v_end.remove_parent()
                        v_new.add_neighbor(v_end)

    path = graph.backtrack(v_start, v_end)

    if plotting:
        plot_path(path, c="#000000", ax=map.ax)

    return path









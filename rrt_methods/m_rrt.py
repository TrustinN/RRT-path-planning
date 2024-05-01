import math
import numpy as np
from .rrt_utils import vertex
from .rrt_utils import graph_init
from utils.rtree.rtree_utils import IndexRecord
from utils.rtree.rstar_tree import RTree
from utils.rtree.rtree_utils import NCircle
from .rrt_utils import rrt_connect_path
from .rrt_utils import upscale, down_sample, up_sample, KPSOptimization


###############################################################################
# RRT Method                                                                  #
###############################################################################

# Idea: When adding a new node, keep it on the map even
# if it might not be immediately connectable.

# Initialize a new graph that contains non-attached nodes

# When adding a node, add it to a separate tree that contains info
# on nodes that are not added to the actual tree.

# When we add a new node to our original tree that is connected,
# search the new tree for the closest node and try to connect

# if we can connect, it causes a domino effect and add the new node to
# the tree.

# Need to bias connecting members further away from parent

def claimExtend(start, vertex, pool, graph, target_graph, map, step_size):

    search_radius = NCircle(center=vertex.value, radius=2 * step_size)
    p = pool.Search(search_radius)
    p.sort(key=lambda x: np.linalg.norm(x.value - vertex.parent.value))

    while p:
        c = p.pop()

        if not map.intersects_line([vertex.value, c.value]):

            pool.Delete(c)
            vertex.add_neighbor(c)
            graph.add_vertex(c)

            search_radius = NCircle(center=c.value, radius=step_size)
            target = target_graph.Search(search_radius)

            if target:
                for t in target:
                    if not map.intersects_line([t.value, c.value]):
                        return True, c, t

            if np.linalg.norm(vertex.value - c.value) > .2 * step_size:
                return claimExtend(start, c, pool, graph, target_graph, map, step_size)

            return False, None, None

    return False, None, None


# Takes in a region that our object can travel in along
# with obstacles and computes the shortest route
# from the starting position to the end position
def rrt_run(map, step_size, max_iter):

    v_start, v_end, t_start, t_end = graph_init(map=map)
    pool = RTree(10, dim=map.dim)
    c1, c2 = None, None
    connect = False

    iter = 0
    while iter < max_iter:

        p_rand = map.sample()

        p_test = IndexRecord(None, p_rand)
        v_near0 = t_start.NearestNeighbor(p_test)[0]
        v_near1 = t_end.NearestNeighbor(p_test)[0]

        p_near0 = v_near0.value
        p_near1 = v_near1.value

        if map.in_free_space(p_rand):
            v_new = vertex(value=p_rand,
                           parent=None,
                           dist_to_root=math.inf,
                           )

            t0 = not map.intersects_line([p_rand, p_near0])
            if t0:

                v_near0.add_neighbor(v_new)
                t_start.add_vertex(v_new)

                # Search in the pool for a vertex to connect to
                connect, c1, c2 = claimExtend(v_start, v_new, pool, t_start, t_end, map, step_size)
                if connect:
                    break

            t1 = not map.intersects_line([p_rand, p_near1])
            if t1:
                if not t0:

                    v_near1.add_neighbor(v_new)
                    t_end.add_vertex(v_new)

                    # Search in the pool for a vertex to connect to
                    connect, c2, c1 = claimExtend(v_end, v_new, pool, t_end, t_start, map, step_size)
                    if connect:
                        break

                else:
                    connect = True
                    c1, c2 = v_new, v_near1
                    break

            if not t0 and not t1:
                pool.Insert(v_new)

            iter += 1

    if not connect:
        path = []

    else:
        path = rrt_connect_path(v_start, v_end, t_start, t_end, c1, c2)
        path = upscale(path, step_size / 2)
        path = down_sample(map, path)
        path = up_sample(map, path, 2 * iter)
        path = down_sample(map, path)
        path = KPSOptimization(path, map, step=20)

    return path, t_start, t_end
















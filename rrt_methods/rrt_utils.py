import math
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from utils.rtree.rstar_tree import RTree
from utils.rtree.rtree_utils import IndexRecord
from utils.rtree.rtree_utils import NCircle


###############################################################################
# Data Structures                                                             #
###############################################################################


class Graph(RTree):
    def __init__(self, vertices=[], num_vertices=0, dim=2):
        super().__init__(10, dim=dim)
        self.vertices = vertices
        self.time = 0

    def add_vertex(self, vertex):
        self.Insert(vertex)
        self.vertices.append(vertex)

    def backtrack(self, start, stop):
        path = [stop.value]
        curr_pos = stop
        while True:
            if curr_pos == start:
                break
            path.insert(0, curr_pos.parent.value)
            curr_pos = curr_pos.parent
        return path

    def backtrack_root(self, stop):
        path = [stop.value]
        curr_pos = stop
        while True:
            if not curr_pos.parent:
                break
            path.insert(0, curr_pos.parent.value)
            curr_pos = curr_pos.parent
        return path

    def clear(self):
        self.vertices = []

    def plot(self, view, branches, leaves):
        super().plot(view, branches, leaves)
        lines = []
        for v in self.vertices:
            if v.parent:
                lines.append(v.value)
                lines.append(v.parent.value)

        if self.dim == 2:
            lp = pg.PlotDataItem(np.array(lines),
                                 connect='pairs',
                                 pen=pg.mkPen("#00a5ff"),
                                 skipFiniteCheck=True,
                                 downsample=10,
                                 width=0.1,
                                 )
            view.addItem(lp)
        if self.dim == 3:
            lp = gl.GLLinePlotItem(pos=np.array(lines),
                                   color=pg.mkColor("#00a5ff"),
                                   width=0.1,
                                   mode='lines')
            lp.setGLOptions("opaque")
            view.addItem(lp)


class vertex(IndexRecord):
    def __init__(self, value=np.array([]),
                 parent=None,
                 dist_to_root=0,
                 ):
        super().__init__(None, value)
        self.value = value
        self.parent = parent
        self.dist_to_root = dist_to_root

    def __hash__(self):
        return hash(self.value.tostring())

    def __eq__(self, other):
        if isinstance(other, self.__class__) and np.array_equal(self.tuple_identifier, other.tuple_identifier):
            return True
        else:
            return False

    def __neq__(self, other):
        return not self.__eq__(other)

    def dist_to(self, other):
        if type(other) is vertex:
            return np.linalg.norm(self.value - other.value)
        elif type(other) is np.ndarray:
            return np.linalg.norm(self.value - other)

    def add_neighbor(self, other):
        other.parent = self
        other.dist_to_root = self.dist_to_root + self.dist_to(other)

    def __repr__(self):
        return f"(value:{self.value})"


###############################################################################
# Search Methods                                                              #
###############################################################################

def search_neighbors(graph, v, r, step_size):

    scope = NCircle(v.value, r * step_size)
    neighbors = graph.Search(scope)
    neighbors.sort(key=lambda x: x.dist_to_root, reverse=True)
    return neighbors

###############################################################################
# Linear Algebra                                                              #
###############################################################################


# Takes a vector v and returns a unit vector in the same direction
def normalize(v):
    return v / np.linalg.norm(v)


###############################################################################
# RRT components                                                              #
###############################################################################


def graph_init(map, connect=False):

    start, end = map.path[0], map.path[1]

    graph0 = Graph(vertices=[], dim=map.dim)
    graph1 = Graph(vertices=[], dim=map.dim)

    v_start = vertex(value=start, parent=None, dist_to_root=0)
    v_end = vertex(value=end, parent=None, dist_to_root=0)

    if not connect:
        v_end.dist_to_root = math.inf

    graph0.add_vertex(v_start)
    graph1.add_vertex(v_end)

    return v_start, v_end, graph0, graph1


def rrt_step(p_rand, v_near, step_size):
    p_near = v_near.value
    p_step = step_size * normalize(p_rand - p_near)
    p_new = p_near + p_step
    return p_new


def rrt_extend_connect(p_rand, p_near, v_near, map, step_size, graph0, graph1):
    c = map.intersections([p_rand, p_near])
    min_dist = np.linalg.norm(p_rand - p_near)
    connect = False
    p_step = normalize(p_rand - p_near)
    expand_iter = 0

    if len(c) != 0:
        for p in c:
            curr_dist = np.linalg.norm(p - p_near)
            if curr_dist < min_dist:
                min_dist = curr_dist

        if min_dist != np.linalg.norm(p_rand - p_near):
            expand_iter = math.floor(min_dist / step_size)

    else:
        expand_iter = math.floor(min_dist / step_size)

    for i in range(expand_iter):
        p_new = step_size * p_step + v_near.value
        v_new = vertex(value=p_new,
                       parent=None,
                       dist_to_root=math.inf
                       )
        v_near.add_neighbor(v_new)
        graph0.add_vertex(v_new)
        prev_parent = v_near
        connect = rrt_connect(v_new, graph0, graph1, map, 5, step_size)
        v_near = v_new
        if connect:
            return connect, v_near, prev_parent
    return connect, v_near, None


# for a vertex v in some graph0, searches in graph1
# for vertices that can be connected to within a radius of
# r * step_size returns a boolean value for if a connection was
# made
def rrt_connect(v, graph0, graph1, map, r, step_size):
    neighbors = search_neighbors(graph1,
                                 v, r,
                                 step_size)

    while neighbors:
        curr_neighbor = neighbors.pop()

        if not map.intersects_line([curr_neighbor.value, v.value]):
            curr_neighbor.add_neighbor(v)
            graph1.add_vertex(v)

            return True

    return False


def rrt_connect_path(v_start, v_end, graph0, graph1, c1, c2):
    path_1 = graph0.backtrack_root(c1)
    path_2 = graph1.backtrack_root(c2)

    path = path_1 + path_2[-1::-1]
    return path


# Looks through graph for vertices within r of vertex
# If distance candidate -> vertex -> root is less than
# distance candidate -> root we reroute.
def rrt_rewire(v, graph, map, r, step_size, end, connect=False):
    neighbors = search_neighbors(graph,
                                 v, r,
                                 step_size
                                 )

    added_to_graph = False
    while neighbors:
        curr_neighbor = neighbors.pop()
        if not map.intersects_line([curr_neighbor.value, v.value]):
            new_parent = curr_neighbor

            new_parent.add_neighbor(v)
            graph.add_vertex(v)
            added_to_graph = True

            while neighbors:
                n = neighbors.pop()
                if n.dist_to_root > v.dist_to(n) + v.dist_to_root:
                    if not map.intersects_line([n.value, v.value]):
                        v.add_neighbor(n)
            if not connect:
                if not map.intersects_line([end.value, v.value]):
                    if v.dist_to_root + v.dist_to(end) < end.dist_to_root:
                        v.add_neighbor(end)

            break

    return added_to_graph


# Looks through graph for vertices within r of vertex
# If distance candidate -> vertex -> root is less than
# distance candidate -> root we reroute.
def rrt_q_rewire(v, graph, map, r, depth, step_size, end, connect=False):
    neighbors = graph.NearestNeighbor(v, 5)[::-1]

    while neighbors:
        curr_neighbor = neighbors.pop()
        if curr_neighbor.dist_to_root != math.inf:
            if not map.intersects_line([curr_neighbor.value, v.value]):
                new_parent = curr_neighbor

                new_parent.add_neighbor(v)
                graph.add_vertex(v)

                for i in range(depth):
                    new_parent = new_parent.parent
                    if new_parent:
                        if not map.intersects_line([new_parent.value, v.value]):
                            new_parent.add_neighbor(v)

                    else:
                        break

                break

    if v.parent:
        if not map.intersects_line([end.value, v.value]):
            if v.dist_to_root + v.dist_to(end) < end.dist_to_root:
                v.add_neighbor(end)


###############################################################################
# Path Smoothing                                                              #
###############################################################################


def down_sample(map, path):
    path_new = []
    if path:
        curr_node = path[0]
        prev_point = curr_node
        path_new.append(curr_node)

        for i in range(len(path) - 1):
            curr_point = path[i + 1]

            if map.intersects_line([curr_node, curr_point]):
                curr_node = prev_point
                path_new.append(prev_point)

            prev_point = curr_point

        path_new.append(path[-1])

    return path_new


def up_sample(map, path, max_iter):
    if path:

        iter = 0
        path_lengths = [np.linalg.norm(path[i + 1] - path[i]) for i in range(len(path) - 1)]
        cumulative_lengths = np.cumsum(path_lengths)
        total_length = sum(path_lengths)

        while iter < max_iter:

            d1 = total_length * np.random.random_sample()
            d2 = total_length * np.random.random_sample()

            # require d1 < d2
            if d2 < d1:
                tmp = d2
                d2 = d1
                d1 = tmp

            # locate the index where d1 occurs given from cumulative sum
            idx = 0
            i, j = 0, 0
            while True:
                if d1 < cumulative_lengths[idx]:
                    i = idx

                    while True:
                        if d2 < cumulative_lengths[idx]:
                            j = idx
                            break

                        idx += 1

                    break
                idx += 1

            a1 = (cumulative_lengths[i] - d1) / path_lengths[i]
            a2 = (cumulative_lengths[j] - d2) / path_lengths[j]

            # interpolation points
            g1 = (1 - a1) * (path[i + 1] - path[i]) + path[i]
            g2 = (1 - a2) * (path[j + 1] - path[j]) + path[j]

            if not map.intersects_line([g1, g2]):
                path = path[:i + 1] + [g1, g2] + path[j + 1:]
                path_lengths = [np.linalg.norm(path[i + 1] - path[i]) for i in range(len(path) - 1)]
                cumulative_lengths = np.cumsum(path_lengths)
                total_length = sum(path_lengths)

            iter += 1

    return path


def tridiag(a, b, c, k1=-1, k2=0, k3=1):
    return np.diag(a, k1) + np.diag(b, k2) + np.diag(c, k3)


def cubic_spline(x, y):
    n = len(x) - 1
    a = y
    h = np.array([x[i + 1] - x[i] for i in range(n)])

    b = np.concatenate((h[:n - 1], np.array([0])))
    m = np.array([1] + [2 * (h[i] + h[i + 1]) for i in range(n - 1)] + [1])
    t = np.concatenate((np.array([0]), h[1:n]))

    A = tridiag(b, m, t)

    f = np.array([0] + [3 * (a[i + 1] - a[i]) / h[i] - 3 * (a[i] - a[i - 1]) / h[i - 1] for i in range(1, n)] + [0])
    c = np.linalg.solve(A, f)
    b = np.array([(a[j + 1] - a[j]) / h[j] - h[j] * (2 * c[j] + c[j + 1]) / 3 for j in range(n)])
    d = np.array([(c[j + 1] - c[j]) / (3 * h[j]) for j in range(n)])

    a = a[:len(a) - 1]
    c = c[:len(c) - 1]
    x = x[:len(x) - 1]
    splines = np.array([a, b, c, d, x]).T

    return splines


def spline_eval(x, y, num=10):
    n = len(x)
    new_path = None
    splines = cubic_spline(x, y)

    for i in range(n - 1):
        cubic = splines[i]
        interval = np.linspace(x[i], x[i + 1], num)
        xj = np.repeat(cubic[4], num)
        diff = interval - xj
        y = cubic[0] + cubic[1] * diff + cubic[2] * diff ** 2 + cubic[3] * diff ** 3
        xy = np.array([interval, y]).T
        if i == 0:
            new_path = xy
        else:
            new_path = np.concatenate((new_path, xy))

    return new_path


def parametric_spline(points, num=10):
    t = np.arange(len(points.T[0]))
    at = []
    for axis in points.T:
        at.append(spline_eval(t, axis, num=num))

    path = np.array([paths.T[1] for paths in at]).T

    return [path[i] for i in range(len(path))]


def KPSOptimization(keypoints, map, num=10):
    kp = keypoints
    path = parametric_spline(np.array(kp), num=num)
    iter = 0
    while True and iter < 5:
        # Check collision
        kpNew = SmoothOptimizer(kp, path, map, num=num)

        if len(kpNew) != len(kp):
            kp = kpNew

        else:
            break

        path = parametric_spline(np.array(kp), num=num)
        iter += 1

    return path


def SmoothOptimizer(keypoints, spoints, map, num=10):
    kp = keypoints
    kpNew = []

    idx = 0
    insert = True
    for i in range(len(spoints) - 1):
        if i // num == idx:
            kpNew.append(kp[idx])
            idx += 1
            insert = True

        p1 = spoints[i]
        p2 = spoints[i + 1]

        if insert and map.intersects_line([p1, p2]):
            kpNew.append((kpNew[-1] + kp[idx]) / 2)
            insert = False

    kpNew.append(kp[-1])
    return kpNew



















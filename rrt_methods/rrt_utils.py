import math
import numpy as np
import matplotlib.pyplot as plt
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from r_trees.r_star_tree import RTree
from r_trees.r_tree_utils import IndexRecord
from r_trees.r_tree_utils import nCircle


###############################################################################
# Plotting                                                                    #
###############################################################################


def plot_path(path, c="#000000"):
    path_x = []
    path_y = []
    for i in range(len(path)):
        idx = i % len(path)
        path_x.append(path[idx][0])
        path_y.append(path[idx][1])
    plt.plot(path_x, path_y, c=c)


def plot_poly(path, c="#000000"):
    path_x = []
    path_y = []
    for i in range(len(path) + 1):
        idx = i % len(path)
        path_x.append(path[idx][0])
        path_y.append(path[idx][1])
    plt.plot(path_x, path_y, c=c)


###############################################################################
# Data Structures                                                             #
###############################################################################


class Graph(RTree):
    def __init__(self, vertices=[], num_vertices=0, plotting=False, view=None, dim=2):
        super().__init__(50, dim=dim, plotting=plotting, view=view)
        self.vertices = vertices
        self.num_vertices = num_vertices

    def make_vertex(self, value=np.array([]),
                    neighbors=[],
                    position=0,
                    parent=None,
                    dist_to_root=0):
        return self.vertex(value=value,
                           neighbors=neighbors,
                           position=position,
                           parent=parent,
                           dist_to_root=dist_to_root,
                           view=self.view)

    def add_vertex(self, vertex):
        self.Insert(vertex)
        self.vertices.append(vertex)
        self.num_vertices += 1

    def backtrack(self, start, stop):
        path = [stop.value]
        curr_pos = stop
        while True:
            if curr_pos.equals(start):
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
        for v in self.vertices:
            v.clear_plots()
            self.num_vertices = 0
            self.vertices = []

    class vertex(IndexRecord):
        def __init__(self, value=np.array([]),
                     neighbors=[],
                     position=0,
                     parent=None,
                     dist_to_root=0,
                     view=None
                     ):
            super().__init__(None, value)
            self.value = value
            self.neighbors = neighbors
            self.position = position
            self.num_neighbors = len(self.neighbors)
            self.parent = parent
            self.dist_to_root = dist_to_root
            self.plots = []
            self.view = view

        def __hash__(self):
            return hash(self.value.tostring())

        def equals(self, other):
            return np.array_equal(self.value, other.value)

        def dist_to(self, other):
            if type(other) is Graph.vertex:
                return np.linalg.norm(self.value - other.value)
            elif type(other) is np.ndarray:
                return np.linalg.norm(self.value - other)

        def add_neighbor(self, other):
            if type(other) is Graph.vertex:
                other.position = self.num_neighbors
                other.parent = self
                self.neighbors.append(other)
                self.num_neighbors += 1

                other.dist_to_root = self.dist_to_root + self.dist_to(other)

                if self.view:
                    p = gl.GLLinePlotItem(pos=np.array([self.value, other.value]),
                                          color=pg.mkColor("#00a5ff"),
                                          )
                    p.setGLOptions("opaque")
                    self.view.addItem(p)
                    self.plots.append(p)

        def remove_neighbor(self, other_pos):
            self.neighbors.pop(other_pos)
            self.num_neighbors -= 1

            for i in range(self.num_neighbors):
                self.neighbors[i].position = i

            if self.view:
                c = self.plots.pop(other_pos)
                self.view.removeItem(c)

        def remove_parent(self):
            if self.parent:
                pos = self.position
                self.parent.remove_neighbor(pos)
                self.position = 0
                self.parent = None

        def clear_plots(self):
            for c in self.plots:
                self.view.removeItem(c)
            self.plots = []

        def __repr__(self):
            return f"(value:{self.value})"


###############################################################################
# Search Methods                                                              #
###############################################################################


# searches graph from start and records all paths
# that satisfy a condition based on the stop variable
def search(f, graph, *args):
    vertices = graph.vertices
    candidates = []
    for v in vertices:
        if f(v, *args):
            candidates.append(v)
    return candidates


def search_visible_neighbors(f, region, obstacles, graph, v, r, step_size):

    scope = nCircle(v.value, r * step_size)
    neighbors = graph.Search(scope)
    visible_neighbors = []

    for n in neighbors:
        if not n.equals(v) and not intersects_objects(region, obstacles, n.value, v.value):
            visible_neighbors.append(n)

    return visible_neighbors


# Given p, return closest point to p in vertices
def closest_point(p, vertices):
    min_dist = math.inf
    closest_pt = None
    for v in vertices:
        curr_dist = v.dist_to(p)
        if curr_dist < min_dist:
            min_dist = curr_dist
            closest_pt = v
    return closest_pt


###############################################################################
# Linear Algebra                                                              #
###############################################################################


# Takes a vector v and returns a unit vector in the same direction
def normalize(v):
    return v / np.linalg.norm(v)


# Returns whether a point lies inside some polygon
def ray_cast(polygon, point):
    num_sides = len(polygon)
    inside = False

    for i in range(num_sides):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % num_sides]
        change_x = p1[0] - p2[0]
        change_y = p1[1] - p2[1]

        if (p1[1] > point[1]) != (p2[1] > point[1]) and change_y != 0:
            inv_slope = change_x / change_y
            y_scale = point[1] - p1[1]
            x_on_line = p1[0] + inv_slope * y_scale

            if x_on_line <= point[0]:
                inside = not inside

    return inside


# Finds intersection points of line segments p2 - p1 and p4 - p3.
# Calculates if there is an intersection within some tolerance and
# returns true or false along with the intersection.
# the intersection is determined by scaling p4 - p3
def intersects(p1, p2, p3, p4, tol):
    l1 = p2 - p1
    l2 = p4 - p3
    b = p3 - p1

    matrix = np.array([[l2[0], -l1[0]], [l2[1], -l1[1]]])
    det = (l2[0] * l1[1]) - (l1[0] * l2[1])
    intersection = False
    intersection_pt = np.array([])

    if not abs(det) < tol:
        inv_matrix = np.linalg.inv(matrix)
        x = np.matmul(inv_matrix, b)

        if -x[0] > 0 and -x[1] > 0 and 1 > -x[0] and 1 > -x[1]:
            intersection_pt = -x[0] * l2 + p3
            intersection = True

    return intersection, intersection_pt


def intersections(region, obstacles, p0, p1):
    points = []
    for j in range(len(region)):
        r_p0 = region[j]
        r_p1 = region[(j + 1) % len(region)]
        intersection, intersection_pt = intersects(r_p0, r_p1, p0, p1, 0.005)
        if intersection:
            points.append(intersection_pt)
    for o in obstacles:
        for j in range(len(o)):
            r_p0 = o[j]
            r_p1 = o[(j + 1) % len(o)]
            intersection, intersection_pt = intersects(r_p0, r_p1, p0, p1, 0.005)
            if intersection:
                points.append(intersection_pt)
    return points


def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


# Return true if line segments AB and CD intersect
def intersect(A, B, C, D):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def intersects_object(region, p0, p1):
    for j in range(len(region)):
        r_p0 = region[j]
        r_p1 = region[(j + 1) % len(region)]
        intersection = intersect(r_p0, r_p1, p0, p1)
        if intersection:
            return True

    return False


# def intersects_objects(region, obstacles, v1, v2):
#     visible = True
#     if intersects_object(region, v1, v2):
#         visible = False
#     for o in obstacles:
#         visible = visible and not intersects_object(o, v1, v2)
#     return not visible

def intersects_objects(region, obstacles, v1, v2):
    line = [v1, v2]
    if region.intersects_line(line):
        return True
    for o in obstacles:
        if o.intersects_line(line):
            return True
    return False

###############################################################################
# Conditions                                                                  #
###############################################################################


def within_dist(p0, p1, dist):
    if type(p0) is np.ndarray:
        return np.linalg.norm(p0 - p1) <= dist
    else:
        return np.linalg.norm(p0.value - p1.value) <= dist


def in_free_space(p, region, obstacles):
    if region.contains_point(p):
        for o in obstacles:
            if o.contains_point(p):
                return False
    else:
        return False
    return True


###############################################################################
# Sampling random point                                                       #
###############################################################################


def sample_circle(r, c):
    r_rand = r * math.sqrt(np.random.random_sample())
    theta = np.random.random_sample() * 2 * math.pi
    return np.array([c[0] + r_rand * math.cos(theta),
                     c[1] + r_rand * math.sin(theta)])


# finds the right, left, north and south bounds
# of a polygon, given its points
def find_bounding_box(polygon):

    p0 = polygon[0]
    min_x, max_x, max_y, min_y = p0[0], p0[0], p0[1], p0[1]

    for i in range(len(polygon)):
        curr_p = polygon[i]

        if curr_p[0] < min_x:
            min_x = curr_p[0]

        if curr_p[0] > max_x:
            max_x = curr_p[0]

        if curr_p[1] > max_y:
            max_y = curr_p[1]

        if curr_p[1] < min_y:
            min_y = curr_p[1]

    return [min_x, max_x, min_y, max_y]


def sample_free(bounds, region, obstacles):
    p_rand = np.array([])
    while True:
        p_rand = bounds.sample()
        if in_free_space(p_rand, region, obstacles):
            break
    return p_rand


def ep_rrt_sample(d, path):
    x_rand = 2 * np.random.random_sample() - 1
    y_rand = 2 * np.random.random_sample() - 1
    scale = 1 / (2 * math.pi)
    x_rand *= d
    y_rand *= d
    v_expand = np.array([x_rand, y_rand])

    num_segments = len(path) - 1
    s_1 = np.random.randint(num_segments)
    s_2 = s_1 + 1

    # idea: approximate sample region with splines/parametrization
    # by intepreting left, right bounds as functions over time
    segment = path[s_2] - path[s_1]
    scale = np.random.random_sample()
    expand_origin = scale * segment + path[s_1]
    return expand_origin + v_expand


###############################################################################
# Density                                                                     #
###############################################################################


# finds area of a polygon with no holes
def find_area(polygon):
    area = 0
    num_sides = len(polygon)
    for i in range(num_sides):
        area += np.cross(polygon[i], polygon[(i + 1) % num_sides])
    return abs(area * .5)


# gets area of a given polygon and a list of holes
def find_area_holes(polygon, holes):
    poly_area = find_area(polygon)
    hole_area = 0
    for h in holes:
        hole_area += find_area(h)
    return poly_area - hole_area


# Calculates expected number of subdivisions of a region given a density
# and the region area
def expected_num(area, density):
    return area / density


###############################################################################
# RRT components                                                              #
###############################################################################


def graph_init(map, connect=False, plotting=False):

    if plotting:
        map.plot()

    start, end = map.path[0], map.path[1]

    graph0 = Graph(plotting=plotting, view=map.view, dim=map.dim)
    graph1 = Graph(plotting=plotting, view=map.view, dim=map.dim)

    v_start = graph0.make_vertex(value=start, neighbors=[])
    v_end = graph1.make_vertex(value=end, neighbors=[], dist_to_root=0)

    if not connect:
        v_end = graph1.make_vertex(value=end, neighbors=[], dist_to_root=math.inf)

    graph0.add_vertex(v_start)
    graph1.add_vertex(v_end)

    return v_start, v_end, graph0, graph1


def rrt_step(p_rand, v_near, step_size):
    p_near = v_near.value
    p_step = step_size * normalize(p_rand - p_near)
    p_new = p_near + p_step
    return p_new


def rrt_extend(p_rand, p_near, v_near, region, obstacles, step_size, graph0, graph1):
    c = intersections(region, obstacles, p_rand, p_near)
    min_dist = np.linalg.norm(p_rand - p_near)

    if len(c) != 0:
        for p in c:
            curr_dist = np.linalg.norm(p - p_near)
            if curr_dist < min_dist:
                min_dist = curr_dist

    p_step = normalize(p_rand - p_near)

    expand_iter = math.floor(min_dist / step_size)
    for i in range(expand_iter):
        p_new = step_size * p_step + v_near.value
        v_new = graph0.make_vertex(value=p_new,
                                   neighbors=[],
                                   position=0,
                                   parent=None,
                                   )
        v_near.add_neighbor(v_new)
        graph0.add_vertex(v_new)
        v_near = v_new
    return v_near


def rrt_extend_connect(p_rand, p_near, v_near, map, step_size, graph0, graph1):
    c = map.intersections([p_rand, p_near])
    min_dist = np.linalg.norm(p_rand - p_near)
    connect = False

    if len(c) != 0:
        for p in c:
            curr_dist = np.linalg.norm(p - p_near)
            if curr_dist < min_dist:
                min_dist = curr_dist

    p_step = normalize(p_rand - p_near)

    expand_iter = math.floor(min_dist / step_size)
    for i in range(expand_iter):
        p_new = step_size * p_step + v_near.value
        v_new = graph0.make_vertex(value=p_new,
                                   neighbors=[],
                                   position=0,
                                   parent=None,
                                   )
        v_near.add_neighbor(v_new)
        graph0.add_vertex(v_new)
        prev_parent = v_near
        connect = rrt_connect(v_new, graph0, graph1, map.region, map.obstacles, 5, step_size)
        v_near = v_new
        if connect:
            return connect, v_near, prev_parent
    return connect, v_near, None


# for a vertex v in some graph0, searches in graph1
# for vertices that can be connected to within a radius of
# r * step_size returns a boolean value for if a connection was
# made
def rrt_connect(v, graph0, graph1, region, obstacles, r, step_size):
    neighbors = search_visible_neighbors(within_dist,
                                         region,
                                         obstacles,
                                         graph1,
                                         v, r,
                                         step_size
                                         )

    if len(neighbors) != 0:
        neighbors[0].add_neighbor(v)
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
def rrt_rewire(v, graph, region, obstacles, r, step_size, end, connect=False):
    neighbors = search_visible_neighbors(within_dist,
                                         region,
                                         obstacles,
                                         graph,
                                         v, r,
                                         step_size
                                         )

    new_parent = None
    added_to_graph = False
    min_dist_to_root = math.inf
    for n in neighbors:
        curr_dist_to_root = n.dist_to(v) + n.dist_to_root
        if curr_dist_to_root < min_dist_to_root:
            new_parent = n
            min_dist_to_root = curr_dist_to_root

    if new_parent:
        new_parent.add_neighbor(v)
        graph.add_vertex(v)
        added_to_graph = True

        for n in neighbors:
            if n.dist_to_root > v.dist_to(n) + v.dist_to_root:
                n.remove_parent()
                v.add_neighbor(n)
        if not connect:
            if not intersects_objects(region, obstacles, end.value, v.value):
                if v.dist_to_root + v.dist_to(end) < end.dist_to_root:
                    end.remove_parent()
                    v.add_neighbor(end)
    return added_to_graph


# Looks through graph for vertices within r of vertex
# If distance candidate -> vertex -> root is less than
# distance candidate -> root we reroute.
def rrt_q_rewire(v, graph, region, obstacles, r, depth, step_size, end, connect=False):
    neighbors = set(search_visible_neighbors(within_dist,
                                             region,
                                             obstacles,
                                             graph,
                                             v, r,
                                             step_size
                                             ))

    added_to_graph = False
    for n in neighbors:
        curr_node = n
        for i in range(depth):
            if curr_node:
                if curr_node.dist_to_root + curr_node.dist_to(v) < v.dist_to_root:
                    if not intersects_objects(region, obstacles, v.value, curr_node.value):
                        v.remove_parent()
                        curr_node.add_neighbor(v)
                        added_to_graph = True
                curr_node = curr_node.parent

    if added_to_graph:
        graph.add_vertex(v)

    if v.parent:
        if not intersects_objects(region, obstacles, end.value, v.value):
            if v.dist_to_root + v.dist_to(end) < end.dist_to_root:
                end.remove_parent()
                v.add_neighbor(end)


















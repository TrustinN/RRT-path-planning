import math
import numpy as np
import matplotlib.pyplot as plt


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


class Graph(object):
    def __init__(self, vertices=[], num_vertices=0):
        self.vertices = vertices
        self.num_vertices = num_vertices

    def add_vertex(self, vertex):
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


class vertex(object):
    def __init__(self, value=np.array([]),
                 neighbors=[],
                 position=0,
                 parent=None,
                 dist_to_root=0,
                 plots=[],
                 enable_plotting=True,
                 ):
        self.value = value
        self.neighbors = neighbors
        self.position = position
        self.num_neighbors = len(self.neighbors)
        self.parent = parent
        self.dist_to_root = dist_to_root
        self.plots = plots
        self.enable_plotting = enable_plotting

    def equals(self, other):
        return np.array_equal(self.value, other.value)

    def dist_to(self, other):
        if type(other) is vertex:
            return np.linalg.norm(self.value - other.value)
        elif type(other) is np.ndarray:
            return np.linalg.norm(self.value - other)

    def add_neighbor(self, other):
        if type(other) is vertex:
            other.position = self.num_neighbors
            other.parent = self
            self.neighbors.append(other)
            self.num_neighbors += 1

            other.dist_to_root = self.dist_to_root + self.dist_to(other)

            if self.enable_plotting:
                p = plt.plot(
                    [self.value[0], other.value[0]],
                    [self.value[1], other.value[1]],
                    c="#7dafe6",
                    )
                self.plots.append(p)

    def remove_neighbor(self, other_pos):
        if self.enable_plotting:
            c = self.plots[other_pos]
            for handle in c:
                handle.remove()
            self.plots.pop(other_pos)

        self.neighbors.pop(other_pos)
        self.num_neighbors -= 1
        for i in range(self.num_neighbors):
            self.neighbors[i].position = i

    def remove_parent(self):
        if self.parent:
            pos = self.position
            self.parent.remove_neighbor(pos)
            self.position = 0
            self.parent = None

    def clear_plots(self):
        for c in self.plots:
            for handle in c:
                handle.remove()
        self.plots = []

    def __repr__(self):
        return f"(value:{self.value})"


# class vertex(object):
#     def __init__(self, value=np.array([]),
#                  neighbors=set(),
#                  parent=None,
#                  dist_to_root=0,
#                  plots=[],
#                  enable_plotting=True,
#                  ):
#         self.value = value
#         self.neighbors = neighbors
#         self.parent = parent
#         self.dist_to_root = dist_to_root
#         self.plots = plots
#         self.enable_plotting = enable_plotting
#
#     def equals(self, other):
#         return np.array_equal(self.value, other.value)
#
#     def dist_to(self, other):
#         if type(other) is vertex:
#             return np.linalg.norm(self.value - other.value)
#         elif type(other) is np.ndarray:
#             return np.linalg.norm(self.value - other)
#
#     def add_neighbor(self, other):
#         if type(other) is vertex:
#             other.parent = self
#             self.neighbors.add(other)
#
#             other.dist_to_root = self.dist_to_root + self.dist_to(other)
#
#             if self.enable_plotting:
#                 p = plt.plot(
#                     [self.value[0], other.value[0]],
#                     [self.value[1], other.value[1]],
#                     c="#7dafe6",
#                     )
#                 self.plots.append(p)
#
#     def remove_neighbor(self, v):
#         # if self.enable_plotting:
#         #     c = self.plots[other_pos]
#         #     for handle in c:
#         #         handle.remove()
#         #     self.plots.pop(other_pos)
#
#         self.neighbors.remove(v)
#
#     def remove_parent(self):
#         if self.parent:
#             self.parent.remove_neighbor(self)
#             self.parent = None
#
#     def clear_plots(self):
#         for c in self.plots:
#             for handle in c:
#                 handle.remove()
#         self.plots = []
#
#     def __repr__(self):
#         return f"(value:{self.value})"


class Ellipse(object):
    def __init__(self, f0, f1, d):
        self.f0 = f0
        self.f1 = f1

        diff_vec = f0 - f1

        self.center = 0.5 * diff_vec + f1
        self.a = d / 2
        self.c = 0.5 * np.linalg.norm(diff_vec)
        self.b = math.sqrt(pow(self.a, 2) - pow(self.c, 2))

        x_vec = np.array([1, 0])
        cos_theta = np.dot(x_vec, diff_vec) / (np.linalg.norm(x_vec) * np.linalg.norm(diff_vec))
        if cos_theta > 1:
            cos_theta = 1
        elif cos_theta < -1:
            cos_theta = -1

        self.angle = np.arccos(cos_theta)


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
    neighbors = search(f, graph, v, r * step_size)
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


def intersects_objects(region, obstacles, v1, v2):
    visible = True
    if intersects_object(region, v1, v2):
        visible = False
    for o in obstacles:
        visible = visible and not intersects_object(o, v1, v2)
    return not visible


###############################################################################
# Conditions                                                                  #
###############################################################################


def within_dist(p0, p1, dist):
    if type(p0) is np.ndarray:
        return np.linalg.norm(p0 - p1) <= dist
    else:
        return np.linalg.norm(p0.value - p1.value) <= dist


def in_free_space(p, region, obstacles):
    valid_pos = False
    if ray_cast(region, p):
        valid_pos = True
        for o in obstacles:
            valid_pos = valid_pos and not ray_cast(o, p)
    return valid_pos


###############################################################################
# Sampling random point                                                       #
###############################################################################

class Sample_Scope():

    def __init__(self, name, scope, map=None):
        self.name = name
        self.overlay = scope


class Map():

    def __init__(self, region, obstacles):
        self.region = region
        self.obstacles = obstacles

    def sample_init(self, name, scope):
        self.scope = Sample_Scope(name, scope)

    def add_path(self, path):
        self.path = path


# Need to create a separate sample space object
class Sampler(object):

    def __init__(self, map):
        self.scope = map.scope.name
        self.region = map.region
        self.obstacles = map.obstacles
        self.overlay = map.scope.overlay

    def sample(self):
        if self.scope == "box":
            return sample_free(self.overlay, self.region, self.obstacles)
        if self.scope == "ellipse":
            return sample_ellipse(self.overlay, buffer=1)


# samples point in a given rectangular area bounded
# west, east, north, and south by some coordinate number
def sample_point(bounds):
    rand_x = (bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0]
    rand_y = (bounds[2] - bounds[3]) * np.random.random_sample() + bounds[3]
    return np.array([rand_x, rand_y])


def sample_circle(r, c):
    r_rand = r * math.sqrt(np.random.random_sample())
    theta = np.random.random_sample() * 2 * math.pi
    return np.array([c[0] + r_rand * math.cos(theta),
                     c[1] + r_rand * math.sin(theta)])


# Sample from and ellipse with foci at p0, p1, defined by distance d
def sample_ellipse(ellipse, buffer=1):
    center = ellipse.center

    r1 = np.random.random_sample()
    r2 = np.random.random_sample()
    x_rand = buffer * ellipse.a * (math.sqrt(r1) + r1) / 2
    y_rand = buffer * ellipse.b * (math.sqrt(r2) + r2) / 2
    theta = np.random.random_sample() * 2 * math.pi
    p = np.array([x_rand * math.cos(theta),
                  y_rand * math.sin(theta)])

    cos_theta = math.cos(-ellipse.angle)
    sin_theta = math.sin(-ellipse.angle)
    rotate = np.array([[cos_theta, -sin_theta],
                      [sin_theta, cos_theta]])

    p = np.dot(rotate, p)
    return np.array([p[0] + center[0], p[1] + center[1]])


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
    return [min_x, max_x, max_y, min_y]


def sample_free(bounds, region, obstacles):
    p_rand = np.array([])
    while True:
        p_rand = sample_point(bounds)
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


def graph_init(start, end, connect=False, plot_tree=True):
    v_start = vertex(value=start, neighbors=[], plots=[], enable_plotting=plot_tree)
    v_end = vertex()
    if connect:
        v_end = vertex(value=end, neighbors=[], plots=[], dist_to_root=0, enable_plotting=plot_tree)
    else:
        v_end = vertex(value=end, neighbors=[], plots=[], dist_to_root=math.inf, enable_plotting=plot_tree)
    graph0 = Graph(vertices=[v_start])
    graph1 = Graph(vertices=[v_end])
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
        v_new = vertex(value=p_new,
                       neighbors=[],
                       position=0,
                       parent=None,
                       plots=[],
                       )
        v_near.add_neighbor(v_new)
        graph0.add_vertex(v_new)
        v_near = v_new
    return v_near


def rrt_extend_connect(p_rand, p_near, v_near, region, obstacles, step_size, graph0, graph1):
    c = intersections(region, obstacles, p_rand, p_near)
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
        v_new = vertex(value=p_new,
                       neighbors=[],
                       position=0,
                       parent=None,
                       plots=[],
                       )
        v_near.add_neighbor(v_new)
        graph0.add_vertex(v_new)
        prev_parent = v_near
        connect = rrt_connect(v_new, graph0, graph1, region, obstacles, 5, step_size)
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

    for n in neighbors:
        curr_node = v
        for i in range(depth):
            if curr_node:
                if curr_node.dist_to_root + curr_node.dist_to(n) < n.dist_to_root:
                    if not intersects_objects(region, obstacles, curr_node.value, n.value):
                        n.remove_parent()
                        curr_node.add_neighbor(n)
                curr_node = curr_node.parent

    if v.parent:
        if not intersects_objects(region, obstacles, end.value, v.value):
            if v.dist_to_root + v.dist_to(end) < end.dist_to_root:
                end.remove_parent()
                v.add_neighbor(end)










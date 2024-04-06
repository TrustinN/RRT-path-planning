import math
import numpy as np
from rrt_methods.rrt_utils import intersects_object
import pyqtgraph.opengl as gl


class Facet():

    def __init__(self, vertices):

        self.vertices = vertices
        self.num_vertices = len(self.vertices)
        self.b = init_point = self.vertices[0]
        self.subspace = np.array([self.vertices[i + 1] - init_point for i in range(len(self.vertices) - 1)]).T
        self.dim = np.linalg.matrix_rank(self.subspace) + 1
        self.o = np.array(self.vertices[:self.dim])
        self.outside_vertices = []
        self.neighbors = []
        self.visited = False
        self.in_conv_poly = True

    def add_neighbor(self, f):
        self.neighbors.append(f)

    def normal(self):
        v1 = self.vertices[1] - self.vertices[0]
        v2 = self.vertices[2] - self.vertices[0]
        n = np.cross(v1, v2)
        return n / np.linalg.norm(n)

    def get_projection(self, p):
        approx = np.linalg.lstsq(self.subspace, p - self.b)[0]
        return p - (np.dot(self.subspace, approx) + self.b)

    def orient(self, p):
        c1 = np.vstack([self.o, p])
        c2 = np.c_[c1, np.ones(self.dim + 1)]
        return np.linalg.det(c2)


def projection_angle(l1, l2):
    cos_theta = np.dot(l1, l2) / (np.linalg.norm(l1) * np.linalg.norm(l2))

    # make sure arccos is defined

    if cos_theta < -1:
        cos_theta = -1
    elif cos_theta > 1:
        cos_theta = 1

    return np.arccos(cos_theta)


# returns the indices of the input vector that
# define the vertices on a convex polygon
def gift_wrap(vertices):

    # first find the left most point
    # we will build the convex polygon from there
    ax_min = math.inf
    idx_min = 0

    for i in range(len(vertices)):
        curr_min = vertices[i][0]

        if curr_min < ax_min:
            ax_min = curr_min
            idx_min = i

    curr_idx = idx_min
    start_pos = vertices[curr_idx]
    curr_pos = start_pos
    curr_side = np.array([0, 1])
    convex_poly = [curr_idx]

    while True:
        min_angle = 2 * math.pi
        min_angle_idx = 0

        # checks through all potential points and calculates
        # the next possible side
        # chooses the side that creates the smallest angle
        # with the previous side.

        for i in range(len(vertices)):
            ray = vertices[i] - curr_pos

            if ray[0] != 0 or ray[1] != 0:
                theta = projection_angle(curr_side, ray)

                if theta < min_angle:
                    min_angle = theta
                    min_angle_idx = i

        # puts the next convex polygon side into convex_poly
        # updates curr_pos, curr_side, curr_idx variables
        # and repeats to find next convex poly side
        curr_side = vertices[min_angle_idx] - curr_pos
        curr_idx = min_angle_idx
        curr_pos = vertices[curr_idx]

        # stop when we have gone around the convex polygon
        # back to our starting position
        if np.array_equal(start_pos, curr_pos):
            break

        convex_poly.append(curr_idx)

    return [vertices[i] for i in convex_poly]


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


def intersections(poly, p0, p1):
    points = []
    for j in range(len(poly)):
        r_p0 = poly[j]
        r_p1 = poly[(j + 1) % len(poly)]
        intersection, intersection_pt = intersects(r_p0, r_p1, p0, p1, 0.005)
        if intersection:
            points.append(intersection_pt)
    return points


class ConvexPoly():

    def __init__(self, faces=[]):

        self.faces = faces
        self.vertices = set([tuple(v) for f in self.faces for v in f.vertices])
        self.vertices = [np.array(v) for v in self.vertices]
        self.x_projection = gift_wrap([np.array([v[1], v[2]]) for v in self.vertices])
        self.y_projection = gift_wrap([np.array([v[0], v[2]]) for v in self.vertices])
        self.z_projection = gift_wrap([np.array([v[0], v[1]]) for v in self.vertices])

    def contains_point(self, p):
        for f in self.faces:
            curr_face = f.vertices
            p2f = curr_face[0] - p
            n = f.normal()
            d = np.dot(p2f, n) / np.linalg.norm(p2f)
            if d < 0:
                return False
        return True

    def intersects_line(self, line):

        x_int = False
        y_int = False
        z_int = False

        if intersects_object(self.x_projection, line[0], line[1]):
            x_int = True

        if intersects_object(self.y_projection, line[0], line[1]):
            y_int = True

        if intersects_object(self.z_projection, line[0], line[1]):
            z_int = True

        return (x_int and y_int) or (x_int and z_int) or (y_int and z_int)

    def intersection_pts(self, line):
        p0 = line[0]
        p1 = line[1]
        x_int = intersections(self.x_projection, p0[1:], p1[1:])
        y_int = intersections(self.x_projection, np.array([p0[0], p0[2]]), np.array([p1[0], p1[2]]))
        z_int = intersections(self.x_projection, p0[0:2], p1[0:2])
        intsects = []

        for points in x_int:
            for p_y in y_int:
                if abs(points[1] - p_y[1]) < 0.04:
                    intsects.append(np.array([p_y[0], points[0], points[1]]))

            for p_z in z_int:
                if abs(points[0] - p_z[1]) < 0.04:
                    intsects.append(np.array([p_z[0], points[0], points[1]]))

        for points in y_int:
            for p_z in z_int:
                if abs(points[0] - p_z[0]) < 0.04:
                    intsects.append(np.array([p_z[0], p_z[1], points[1]]))

        return intsects

    def plot(self, view):
        faces = np.array([f.vertices for f in self.faces])
        vertices = [f[i] for f in faces for i in range(3)]
        indices = np.arange(3 * len(faces)).reshape(len(faces), 3)

        md = gl.MeshData(vertexes=vertices, faces=indices)
        colors = np.ones((md.faceCount(), 4), dtype=float)
        colors[:, 3] = 0.3
        colors[:, 2] = np.linspace(0, 1, colors.shape[0])

        md.setFaceColors(colors=colors)
        m1 = gl.GLMeshItem(meshdata=md, smooth=False, shader='balloon')
        m1.setGLOptions('additive')

        view.addItem(m1)




































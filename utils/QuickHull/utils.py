import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rrt_methods.rrt_utils import intersects_object


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

    def plot(self, ax):
        self.plots = [ax.add_collection3d(Poly3DCollection([self.vertices], alpha=1))]
        for v in self.vertices:
            self.plots.append(ax.scatter(v[0], v[1], v[2], s=10, edgecolor='none'))

    def rm_plot(self):
        for p in self.plots:
            p.remove()

    def __eq__(self, other):
        if isinstance(other, Facet):
            for i in range(len(self.vertices)):
                if not np.array_equal(self.vertices[i], other.vertices[i]):
                    return False
            return True
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return f"val: {self.vertices}"

    def __repr__(self):
        return f"val: {self.vertices}"


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

        return x_int and y_int and z_int

    def plot(self, ax):
        for f in self.faces:
            f.plot(ax=ax)

    def animate(self):
        if self.plotting:
            for angle in range(0, 1000, 2):

                self.ax.view_init(elev=angle + math.sin(1 / (angle + 1)) / 5, azim=.7 * angle, roll=.8 * angle)
                plt.draw()
                plt.pause(.001)

            plt.show()












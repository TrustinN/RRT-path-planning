import numpy as np
from colorutils import Color
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from utils.rtree.rtree_utils import Rect
from utils.rtree.rtree_utils import Cube
from utils.rtree.rtree_utils import IndexRecord


# Finds intersection points of line segments p2 - p1 and p4 - p3.
# Calculates if there is an intersection within some tolerance and
# returns true or false along with the intersection.
# the intersection is determined by scaling p4 - p3
def intersects(p1, p2, p3, p4, tol=0.005):
    l1 = p2 - p1
    l2 = p4 - p3
    b = p3 - p1

    matrix = np.array([[l2[0], -l1[0]], [l2[1], -l1[1]]])
    det = (l2[0] * l1[1]) - (l1[0] * l2[1])

    if not abs(det) < tol:
        inv_matrix = np.linalg.inv(matrix)
        x = np.matmul(inv_matrix, b)

        if -x[0] > 0 and -x[1] > 0 and 1 > -x[0] and 1 > -x[1]:
            intersection_pt = -x[0] * l2 + p3
            return intersection_pt

    return None


class Facet(IndexRecord):

    def __init__(self, vertices):

        self.vertices = vertices
        self.num_vertices = len(self.vertices)

        if self.num_vertices >= 0:
            if len(self.vertices[0]) == 2:
                self.rotate = np.array([[0, -1],
                                        [1, 0]])
                n = np.dot(self.rotate, self.vertices[1] - self.vertices[0])
                self.normal = n / np.linalg.norm(n)
                self.bound = Rect.combine_points(self.vertices)
                super().__init__(self.bound, self.vertices[0])

            elif len(self.vertices[0]) == 3:
                if self.num_vertices >= 3:

                    v1 = self.vertices[1] - self.vertices[0]
                    v2 = self.vertices[2] - self.vertices[0]
                    n = np.cross(v1, v2)
                    self.normal = n / np.linalg.norm(n)
                    self.bound = Cube.combine_points(self.vertices)
                    self.bound = Cube([self.bound.bound[0] - 1, self.bound.bound[1] + 1,
                                       self.bound.bound[2] - 1, self.bound.bound[3] + 1,
                                       self.bound.bound[4] - 1, self.bound.bound[5] + 1])
                    super().__init__(self.bound, self.vertices[0])

        self.b = init_point = self.vertices[0]
        self.subspace = np.array([self.vertices[i + 1] - init_point for i in range(len(self.vertices) - 1)]).T
        self.dim = np.linalg.matrix_rank(self.subspace) + 1
        self.o = np.array(self.vertices[:self.dim])
        self.outside_vertices = []
        self.neighbors = []
        self.visited = False
        self.in_conv_poly = True
        self.triangles = []

    def add_neighbor(self, f):
        self.neighbors.append(f)

    def get_projection(self, p):
        approx = np.linalg.lstsq(self.subspace, p - self.b, rcond=None)[0]
        return p - (np.dot(self.subspace, approx) + self.b)

    def orient(self, p):
        return np.dot(self.normal, self.b - p)

    def intersects_line(self, start, end):
        if self.dim == 2:
            def ccw(A, B, C):
                return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

            # Return true if line segments AB and CD intersect
            def intersect(A, B, C, D):
                return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

            return intersect(start, end, self.vertices[0], self.vertices[1])

        elif self.dim == 3:
            vec = end - start
            if np.sign(self.orient(start)) != np.sign(self.orient(end)):

                d = np.dot(self.normal, self.vertices[0])
                na = np.dot(self.normal, start)
                nv = np.dot(self.normal, vec)
                t = (d - na) / nv
                if 0 <= t <= 1:

                    inter = t * vec + start
                    iter = 0
                    num_neighbors = len(self.neighbors)
                    if self.bound.contains_point(inter):
                        while iter < num_neighbors:

                            if self.neighbors[iter].orient(inter) < 0:
                                break
                            iter += 1

                        if iter == num_neighbors:
                            return True

            return False

    def intersections(self, line):
        if self.dim == 2:
            return intersects(self.vertices[0], self.vertices[1], line[0], line[1])

        elif self.dim == 3:
            start = line[0]
            end = line[1]
            vec = end - start
            if np.sign(self.orient(start)) != np.sign(self.orient(end)):

                d = np.dot(self.normal, self.vertices[0])
                na = np.dot(self.normal, start)
                nv = np.dot(self.normal, vec)
                t = (d - na) / nv
                if 0 <= t <= 1:

                    inter = t * vec + start
                    iter = 0
                    num_neighbors = len(self.neighbors)
                    while iter < num_neighbors:

                        if self.neighbors[iter].orient(inter) < 0:
                            break
                        iter += 1

                    if iter == num_neighbors:
                        return inter

            return None

    def plot(self, color, view):
        if self.dim == 2:
            line = pg.PlotDataItem(np.array([self.vertices[0],
                                             self.vertices[1]
                                             ]),
                                   connect="pairs", pen=pg.mkPen(color))
            self.p = line
            view.addItem(line)

        elif self.dim == 3:

            md = gl.MeshData(vertexes=self.vertices, faces=np.array([[0, 1, 2]]))
            c = Color(web=color)
            rgb = c.rgb
            p0, p1, p2 = rgb[0], rgb[1], rgb[2]
            colors = np.ones((md.faceCount(), 4), dtype=float)
            colors[:, 3] = 0.3
            colors[:, 2] = np.linspace(p2/255, 1, colors.shape[0])
            colors[:, 1] = np.linspace(p1/255, 1, colors.shape[0])
            colors[:, 0] = np.linspace(p0/255, 1, colors.shape[0])

            md.setFaceColors(colors=colors)
            m1 = gl.GLMeshItem(meshdata=md, smooth=False, shader='shaded')
            m1.setGLOptions('opaque')

            self.p = m1
            view.addItem(m1)


class ConvexPoly():

    def __init__(self, faces=[]):

        self.faces = faces
        self.num_calc = 0

        copy = faces[:]
        for f in copy:
            if not f.in_conv_poly:
                self.faces.remove(f)

        if self.faces:
            if self.faces[0].dim == 2:
                self.bound = Rect.combine([f.bound for f in self.faces])

            elif self.faces[0].dim == 3:
                self.bound = Cube.combine([f.bound for f in self.faces])

        self.is_visible_obstacle = True

    def contains_point(self, p):
        if self.bound.contains_point(p):
            for f in self.faces:
                curr_face = f.vertices
                p2f = curr_face[0] - p
                n = f.normal
                d = np.dot(p2f, n) / np.linalg.norm(p2f)
                if d < 0:
                    return False
            return True
        else:
            return False

    def plot(self, view):
        for f in self.faces:
            f.plot("#ffffff", view)

    def rm_plot(self):
        for f in self.faces:
            f.rm_plot()






































#

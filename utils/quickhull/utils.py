import math
import numpy as np
from colorutils import Color
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from utils.rtree.rtree_utils import Rect
from utils.rtree.rtree_utils import Cube
from utils.rtree.rtree_utils import IndexRecord


class Facet(IndexRecord):

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
        self.rotate = np.array([[0, -1],
                                [1, 0]])

        if self.num_vertices >= 0:
            if len(self.vertices[0]) == 2:
                n = np.dot(self.rotate, self.vertices[1] - self.vertices[0])
                self.normal = n / np.linalg.norm(n)
                min_x, min_y = math.inf, math.inf
                max_x, max_y = -math.inf, -math.inf

                for v in self.vertices:
                    c_x = v[0]
                    c_y = v[1]

                    if c_x < min_x:
                        min_x = c_x
                    if c_x > max_x:
                        max_x = c_x

                    if c_y < min_y:
                        min_y = c_y
                    if c_y > max_y:
                        max_y = c_y

                self.bound = Rect([min_x, max_x, min_y, max_y])
                super().__init__(self.bound, self.vertices[0])

            elif len(self.vertices[0]) == 3:
                if self.num_vertices >= 3:

                    v1 = self.vertices[1] - self.vertices[0]
                    v2 = self.vertices[2] - self.vertices[0]
                    n = np.cross(v1, v2)
                    self.normal = n / np.linalg.norm(n)

                    min_x, min_y, min_z = math.inf, math.inf, math.inf
                    max_x, max_y, max_z = -math.inf, -math.inf, -math.inf

                    for v in self.vertices:
                        c_x = v[0]
                        c_y = v[1]
                        c_z = v[2]
                        if c_x < min_x:
                            min_x = c_x
                        if c_x > max_x:
                            max_x = c_x

                        if c_y < min_y:
                            min_y = c_y
                        if c_y > max_y:
                            max_y = c_y

                        if c_z < min_z:
                            min_z = c_z
                        if c_z > max_z:
                            max_z = c_z
                    self.bound = Cube([min_x, max_x, min_y, max_y, min_z, max_z])
                    super().__init__(self.bound, self.vertices[0])

    def add_neighbor(self, f):
        self.neighbors.append(f)

    def get_projection(self, p):
        approx = np.linalg.lstsq(self.subspace, p - self.b)[0]
        return p - (np.dot(self.subspace, approx) + self.b)

    def orient(self, p):
        return np.dot(self.normal, self.b - p)

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
        if self.faces:
            if self.faces[0].dim == 2:
                self.bound = Rect.combine([f.bound for f in self.faces])

            elif self.faces[0].dim == 3:
                self.bound = Cube.combine([f.bound for f in self.faces])

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

    def intersects_line(self, line):
        start = line[0]
        end = line[1]
        vec = end - start
        l_bound = Cube([min(start[0], end[0]), max(start[0], end[0]), min(start[1], end[1]), max(start[1], end[1]), min(start[2], end[2]), max(start[2], end[2])])

        if l_bound.overlap(self.bound) > 0:
            for f in self.faces:
                if l_bound.overlap(f.bound) > 0:
                    if np.sign(f.orient(start)) != np.sign(f.orient(end)):

                        d = np.dot(f.normal, f.vertices[0])
                        na = np.dot(f.normal, start)
                        nv = np.dot(f.normal, vec)
                        t = (d - na) / nv
                        if 0 <= t <= 1:

                            inter = t * vec + start
                            iter = 0
                            if f.bound.contains_point(inter):
                                while iter < 3:
                                    if f.neighbors[iter].orient(inter) < 0:
                                        break
                                    iter += 1

                                if iter == 3:
                                    return True

        return False

    def intersections(self, line):
        start = line[0]
        end = line[1]
        l_bound = Cube([min(start[0], end[0]), max(start[0], end[0]), min(start[1], end[1]), max(start[1], end[1]), min(start[2], end[2]), max(start[2], end[2])])
        vec = end - start
        inters = []

        if l_bound.overlap(self.bound) > 0:
            for f in self.faces:
                if l_bound.overlap(f.bound) > 0:
                    if np.sign(f.orient(start)) != np.sign(f.orient(end)):

                        d = np.dot(f.normal, f.vertices[0])
                        na = np.dot(f.normal, start)
                        nv = np.dot(f.normal, vec)
                        t = (d - na) / nv
                        if 0 <= t <= 1:

                            inter = t * vec + start
                            iter = 0
                            while iter < 3:
                                if f.neighbors[iter].orient(inter) < 0:
                                    break
                                iter += 1

                            if iter == 3:
                                inters.append(inter)

        return inters

    def plot(self, view):
        for f in self.faces:
            f.plot("#ffffff", view)

    def rm_plot(self):
        for f in self.faces:
            f.rm_plot()





































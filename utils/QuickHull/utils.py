import numpy as np
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
        if self.num_vertices >= 3:
            v1 = self.vertices[1] - self.vertices[0]
            v2 = self.vertices[2] - self.vertices[0]
            n = np.cross(v1, v2)
            self.normal = n / np.linalg.norm(n)

    def add_neighbor(self, f):
        self.neighbors.append(f)

    def get_projection(self, p):
        approx = np.linalg.lstsq(self.subspace, p - self.b)[0]
        return p - (np.dot(self.subspace, approx) + self.b)

    def orient(self, p):
        c1 = np.vstack([self.o, p])
        c2 = np.c_[c1, np.ones(self.dim + 1)]
        return np.linalg.det(c2)


class ConvexPoly():

    def __init__(self, faces=[]):

        self.faces = faces

    def contains_point(self, p):
        for f in self.faces:
            curr_face = f.vertices
            p2f = curr_face[0] - p
            n = f.normal
            d = np.dot(p2f, n) / np.linalg.norm(p2f)
            if d < 0:
                return False
        return True

    def intersects_line(self, line):
        start = line[0]
        end = line[1]
        vec = end - start
        for f in self.faces:
            if np.sign(f.orient(start)) != np.sign(f.orient(end)):
                d = -np.dot(f.normal, f.vertices[0])
                na = np.dot(f.normal, start)
                nv = np.dot(f.normal, vec)
                t = (d - na) / nv
                if 0 <= t <= 1:
                    inter = t * vec + start
                    num = 0
                    for fn in f.neighbors:
                        if fn.orient(inter) < 0:
                            num += 1
                    if num == 3:
                        return True
        return False

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




































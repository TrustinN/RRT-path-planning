import math
import numpy as np
from colorutils import Color
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import PyQt5


class Bound:

    def __init__(self, dim):
        self.dim = dim

    def margin(self):
        return

    def expand(self):
        return

    def overlap(b1, b2):
        return

    def plot(self, c):
        return

    def rm_plot(self):
        return


class NCircle(Bound):

    def __init__(self, center, radius):
        super().__init__(len(center))
        self.center = center
        self.radius = radius

    def overlap(self, other):

        if type(other) is NCircle:
            return np.linalg.norm(self.center - other.center) >= self.radius + other.radius

        elif type(other) is Rect:
            return Rect.get_dist(other, self.center) <= self.radius

        elif type(other) is Cube:
            return Cube.get_dist(other, self.center) <= self.radius

    def plot(self, color, view):

        if self.dim == 2:
            circle = PyQt5.QtWidgets.QGraphicsEllipseItem(self.center[0] - self.radius,
                                                          self.center[1] - self.radius,
                                                          2 * self.radius,
                                                          2 * self.radius)

            circle.setBrush(pg.mkBrush(color))
            view.addItem(circle)

        elif self.dim == 3:
            md = gl.MeshData.sphere(rows=10, cols=20, radius=self.radius)
            c = Color(web=color)
            rgb = c.rgb
            p0, p1, p2 = rgb[0], rgb[1], rgb[2]
            colors = np.ones((md.faceCount(), 4), dtype=float)
            colors[:, 3] = 0.2
            colors[:, 2] = np.linspace(p2/255, 1, colors.shape[0])
            colors[:, 1] = np.linspace(p1/255, 1, colors.shape[0])
            colors[:, 0] = np.linspace(p0/255, 1, colors.shape[0])

            md.setFaceColors(colors=colors)
            m1 = gl.GLMeshItem(
                meshdata=md,
                smooth=True,
                shader="balloon",
                glOptions="additive",
            )

            m1.translate(self.center[0], self.center[1], self.center[2])
            view.addItem(m1)

    def contains(self, other):

        bound = other.bound

        if type(other) is Rect:
            for i in range(2):
                for j in range(2):
                    corner = np.array([bound[i], bound[j + 2]])

                    if np.linalg.norm(corner - self.center) > self.radius:
                        return False

            return True

        if type(other) is Cube:
            for i in range(2):
                for j in range(2):
                    for k in range(2):
                        corner = np.array([bound[i], bound[j + 2], bound[k + 4]])

                        if np.linalg.norm(corner - self.center) > self.radius:
                            return False

            return True


class Rect(Bound):

    def __init__(self, bound=[]):

        super().__init__(2)
        self.bound = bound

        if bound:

            self.min_x = bound[0]
            self.max_x = bound[1]
            self.min_y = bound[2]
            self.max_y = bound[3]

            self.length = self.max_x - self.min_x
            self.width = self.max_y - self.min_y
            self.vol = self.width * self.length

            self.center_x = self.min_x + self.length / 2
            self.center_y = self.min_y + self.width / 2
            self.center = np.array([self.center_x, self.center_y])

            self.p_obj = None

    # returns perimeter
    def margin(self):
        return 2 * (self.length + self.width)

    def contains(self, other):
        return (self.min_x <= other.min_x) and (self.max_x >= other.max_x) and (self.min_y <= other.min_y) and (self.max_y >= other.max_y)

    def contains_point(self, p):

        if not (self.min_x <= p[0] <= self.max_x):
            return False

        if not (self.min_y <= p[1] <= self.max_y):
            return False

        return True

    # returns bounds and area
    def expand(b1, b2):

        min_x = min(b1.min_x, b2.min_x)
        max_x = max(b1.max_x, b2.max_x)
        min_y = min(b1.min_y, b2.min_y)
        max_y = max(b1.max_y, b2.max_y)

        return [min_x, max_x, min_y, max_y]

    def expand_vol(b1, b2):

        bound = Rect.expand(b1, b2)
        return (bound[1] - bound[0]) * (bound[3] - bound[2])

    def combine(bounds):

        min_x, max_x, min_y, max_y = math.inf, -math.inf, math.inf, -math.inf

        for b in bounds:
            if b.min_x < min_x:
                min_x = b.min_x

            if b.max_x > max_x:
                max_x = b.max_x

            if b.min_y < min_y:
                min_y = b.min_y

            if b.max_y > max_y:
                max_y = b.max_y

        return Rect([min_x, max_x, min_y, max_y])

    def combine_points(points):

        min_x, max_x, min_y, max_y = math.inf, -math.inf, math.inf, -math.inf

        for p in points:
            x = p[0]
            y = p[1]
            if x < min_x:
                min_x = x

            if x > max_x:
                max_x = x

            if y < min_y:
                min_y = y

            if y > max_y:
                max_y = y

        return Rect([min_x, max_x, min_y, max_y])

    # returns overlap area of two bounds
    def overlap(self, other):

        l_sum = .5 * (self.length + other.length)
        w_sum = .5 * (self.width + other.width)

        x_dist = abs(self.center_x - other.center_x)
        y_dist = abs(self.center_y - other.center_y)

        overlap_x = l_sum - x_dist
        overlap_y = w_sum - y_dist

        if overlap_x <= 0 or overlap_y <= 0:
            return 0

        else:
            return overlap_x * overlap_y

    def get_dist(b, point):

        bound = b.bound
        x_dist = min(abs(point[0] - bound[0]), abs(point[0] - bound[1]))
        y_dist = min(abs(point[1] - bound[2]), abs(point[1] - bound[3]))

        btw_x = bound[0] <= point[0] <= bound[1]
        btw_y = bound[2] <= point[1] <= bound[3]

        if btw_x and btw_y:
            return 0

        if btw_x:
            return y_dist

        if btw_y:
            return x_dist

        return math.sqrt(x_dist ** 2 + y_dist ** 2)

    def plot(self, color, view):
        self.p_obj = pg.PlotDataItem(np.array([np.array([self.min_x, self.min_y]),
                                               np.array([self.min_x, self.max_y]),

                                               np.array([self.min_x, self.max_y]),
                                               np.array([self.max_x, self.max_y]),

                                               np.array([self.max_x, self.max_y]),
                                               np.array([self.max_x, self.min_y]),

                                               np.array([self.max_x, self.min_y]),
                                               np.array([self.min_x, self.min_y]),
                                               ]),
                                     connect="pairs", pen=pg.mkPen(color))

        self.view = view
        view.addItem(self.p_obj)

    def rm_plot(self):
        if self.p_obj:
            self.view.removeItem(self.p_obj)
        self.p_obj = None

    def __str__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y]}"

    def __repr__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y]}"


class Cube(Bound):

    def __init__(self, bound=[]):

        super().__init__(3)
        self.bound = bound
        self.hull = None

        if bound:

            self.min_x = bound[0]
            self.max_x = bound[1]
            self.min_y = bound[2]
            self.max_y = bound[3]
            self.min_z = bound[4]
            self.max_z = bound[5]

            self.length = self.max_x - self.min_x
            self.width = self.max_y - self.min_y
            self.height = self.max_z - self.min_z

            self.vol = self.width * self.length * self.height
            self.center_x = self.min_x + self.length / 2
            self.center_y = self.min_y + self.width / 2
            self.center_z = self.min_z + self.height / 2
            self.center = np.array([self.center_x, self.center_y, self.center_z])

            self.p_obj = None

    # returns perimeter measure
    def margin(self):
        return self.length + self.width + self.height

    def contains(self, other):

        x_check = (self.min_x <= other.min_x) and (self.max_x >= other.max_x)
        y_check = (self.min_y <= other.min_y) and (self.max_y >= other.max_y)
        z_check = (self.min_z <= other.min_z) and (self.max_z >= other.max_z)

        return x_check and y_check and z_check

    def contains_point(self, p):

        if not (self.min_x <= p[0] <= self.max_x):
            return False

        if not (self.min_y <= p[1] <= self.max_y):
            return False

        if not (self.min_z <= p[2] <= self.max_z):
            return False

        return True

    # returns bounds and area
    def expand(b1, b2):

        min_x = min(b1.min_x, b2.min_x)
        max_x = max(b1.max_x, b2.max_x)

        min_y = min(b1.min_y, b2.min_y)
        max_y = max(b1.max_y, b2.max_y)

        min_z = min(b1.min_z, b2.min_z)
        max_z = max(b1.max_z, b2.max_z)

        return [min_x, max_x, min_y, max_y, min_z, max_z]

    def expand_vol(b1, b2):

        bound = Cube.expand(b1, b2)
        return (bound[1] - bound[0]) * (bound[3] - bound[2]) * (bound[5] - bound[4])

    def combine(bounds):

        min_x, max_x, min_y, max_y, min_z, max_z = math.inf, -math.inf, math.inf, -math.inf, math.inf, -math.inf

        for b in bounds:
            if b.min_x < min_x:
                min_x = b.min_x
            if b.max_x > max_x:
                max_x = b.max_x

            if b.min_y < min_y:
                min_y = b.min_y
            if b.max_y > max_y:
                max_y = b.max_y

            if b.min_z < min_z:
                min_z = b.min_z
            if b.max_z > max_z:
                max_z = b.max_z

        return Cube([min_x, max_x, min_y, max_y, min_z, max_z])

    def combine_points(points):

        min_x, max_x, min_y, max_y, min_z, max_z = math.inf, -math.inf, math.inf, -math.inf, math.inf, -math.inf

        for p in points:
            x = p[0]
            y = p[1]
            z = p[2]

            if x < min_x:
                min_x = x

            if x > max_x:
                max_x = x

            if y < min_y:
                min_y = y

            if y > max_y:
                max_y = y

            if z < min_z:
                min_z = z

            if z > max_z:
                max_z = z

        return Cube([min_x, max_x, min_y, max_y, min_z, max_z])

    # returns overlap area of two bounds
    def overlap(self, other):

        l_sum = .5 * (self.length + other.length)
        w_sum = .5 * (self.width + other.width)
        h_sum = .5 * (self.height + other.height)

        x_dist = abs(self.center_x - other.center_x)
        y_dist = abs(self.center_y - other.center_y)
        z_dist = abs(self.center_z - other.center_z)

        overlap_x = l_sum - x_dist
        overlap_y = w_sum - y_dist
        overlap_z = h_sum - z_dist

        if overlap_x <= 0 or overlap_y <= 0 or overlap_z <= 0:
            return 0

        else:
            return overlap_x * overlap_y * overlap_z

    def get_dist(b, point):

        bound = b.bound
        x_dist = min(abs(point[0] - bound[0]), abs(point[0] - bound[1]))
        y_dist = min(abs(point[1] - bound[2]), abs(point[1] - bound[3]))
        z_dist = min(abs(point[2] - bound[4]), abs(point[2] - bound[5]))

        btw_x = bound[0] <= point[0] <= bound[1]
        btw_y = bound[2] <= point[1] <= bound[3]
        btw_z = bound[4] <= point[2] <= bound[5]

        if btw_x and btw_y and btw_z:
            return 0

        if btw_x and btw_y:
            return z_dist

        if btw_x and btw_z:
            return y_dist

        if btw_y and btw_z:
            return x_dist

        if btw_x:
            return math.sqrt(y_dist ** 2 + z_dist ** 2)

        if btw_y:
            return math.sqrt(x_dist ** 2 + z_dist ** 2)

        if btw_z:
            return math.sqrt(x_dist ** 2 + y_dist ** 2)

        return math.sqrt(x_dist ** 2 + y_dist ** 2 + z_dist ** 2)

    def plot(self, color, view):
        vertices = [np.array([self.min_x, self.min_y, self.min_z]),  # 0
                    np.array([self.max_x, self.min_y, self.min_z]),  # 1
                    np.array([self.min_x, self.max_y, self.min_z]),  # 2
                    np.array([self.min_x, self.min_y, self.max_z]),  # 3
                    np.array([self.max_x, self.max_y, self.min_z]),  # 4
                    np.array([self.min_x, self.max_y, self.max_z]),  # 5
                    np.array([self.max_x, self.min_y, self.max_z]),  # 6
                    np.array([self.max_x, self.max_y, self.max_z]),  # 7
                    ]

        md = gl.MeshData(vertexes=vertices,
                         faces=np.array([
                             # bottom plane
                             [0, 1, 4],
                             [0, 4, 2],

                             # left plane
                             [0, 1, 6],
                             [0, 6, 3],

                             # back plane
                             [0, 2, 5],
                             [0, 5, 3],

                             # right plane
                             [2, 7, 5],
                             [2, 4, 7],

                             # top plane
                             [3, 7, 5],
                             [3, 6, 7],

                             # front plane
                             [4, 6, 1],
                             [4, 7, 6]
                             ]),
                         )

        c = Color(web=color)
        rgb = c.rgb
        p0, p1, p2 = rgb[0], rgb[1], rgb[2]
        colors = np.ones((md.faceCount(), 4), dtype=float)
        colors[:, 3] = 0.2
        colors[:, 2] = np.linspace(p2/255, 1, colors.shape[0])
        colors[:, 1] = np.linspace(p1/255, 1, colors.shape[0])
        colors[:, 0] = np.linspace(p0/255, 1, colors.shape[0])

        md.setFaceColors(colors=colors)
        m1 = gl.GLMeshItem(meshdata=md, smooth=False, shader='shaded')
        m1.setGLOptions('additive')
        self.p_obj = m1
        view.addItem(m1)

    def rm_plot(self, view):
        if self.p_obj:
            view.removeItem(self.p_obj)

    def __str__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z]}"

    def __repr__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z]}"


class Entry:

    def __init__(self, bound):

        self.bound = bound


class IndexRecord(Entry):

    def __init__(self, bound, tuple_identifier):

        self.dim = len(tuple_identifier)

        if not bound:
            if self.dim == 2:
                bound = Rect([tuple_identifier[i // 2] for i in range(2 * self.dim)])

            elif self.dim == 3:
                bound = Cube([tuple_identifier[i // 2] for i in range(2 * self.dim)])

        super().__init__(bound)
        self.tuple_identifier = tuple_identifier

    def plot(self, color, view):

        if self.dim == 2:
            self.p = pg.ScatterPlotItem(pos=np.array([self.tuple_identifier]), size=3)
            self.p.setBrush(color)

        elif self.dim == 3:
            self.p = gl.GLScatterPlotItem(pos=np.array([self.tuple_identifier]), size=5, color=pg.mkColor(color))
            self.p.setGLOptions("additive")

        self.view = view
        view.addItem(self.p)

    def rm_plot(self):
        if self.view:
            self.view.removeItem(self.p)

    def __str__(self):
        return f"val: {self.tuple_identifier}"

    def __repr__(self):
        return f"val: {self.tuple_identifier}"

    def __eq__(self, other):
        if isinstance(other, self.__class__) and np.array_equal(self.tuple_identifier, other.tuple_identifier):
            return True
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)


class IndexPointer(Entry):

    def __init__(self, bound, pointer):

        super().__init__(bound)
        self.pointer = pointer

    def update(self, bound):
        self.bound = bound

    def __str__(self):
        return "pt " + f"{self.bound} -> {self.pointer}"

    def __repr__(self):
        return "pt " + f"{self.bound} -> {self.pointer}"















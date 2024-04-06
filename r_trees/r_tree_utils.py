import math
import numpy as np
import matplotlib.pyplot as plt


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


class nCircle(Bound):

    def __init__(self, center, radius):
        super().__init__(len(center))
        self.center = center
        self.radius = radius

    def get_sphere():

        phi, theta = np.mgrid[0:np.pi:100j, 0:2 * np.pi:100j]
        x = np.cos(theta) * np.sin(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(phi)

        return x, y, z

    def overlap(self, other):

        if type(other) is nCircle:
            return np.linalg.norm(self.center - other.center) >= self.radius + other.radius

        elif type(other) is Rect:
            return Rect.get_dist(other, self.center) <= self.radius

        elif type(other) is Cube:
            return Cube.get_dist(other, self.center) <= self.radius

    def plot(self, c, ax):

        if self.dim == 2:

            cc = plt.Circle((self.center[0], self.center[1]), self.radius, fill=False)
            ax.add_artist(cc)

        elif self.dim == 3:

            x, y, z = nCircle.get_sphere()
            ax.plot_surface(self.radius * x + self.center[0],
                            self.radius * y + self.center[1],
                            self.radius * z + self.center[2],
                            alpha=0.08,
                            shade=False,
                            )

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

    def plot(self, c, ax):
        self.p_obj = ax.plot([self.min_x, self.min_x, self.max_x, self.max_x, self.min_x],
                             [self.min_y, self.max_y, self.max_y, self.min_y, self.min_y],
                             c=c, linewidth=.5)

    def rm_plot(self):
        if self.p_obj:
            for handle in self.p_obj:
                handle.remove()
        self.p_obj = None

    def __str__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y]}"

    def __repr__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y]}"


class Cube(Bound):

    def __init__(self, bound=[]):

        super().__init__(3)
        self.bound = bound

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

    def get_cube():

        phi = np.arange(1, 10, 2) * np.pi / 4
        Phi, Theta = np.meshgrid(phi, phi)

        x = np.cos(Phi) * np.sin(Theta)
        y = np.sin(Phi) * np.sin(Theta)
        z = np.cos(Theta) / np.sqrt(2)

        return x, y, z

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

    def plot(self, c, ax):
        x, y, z = Cube.get_cube()
        self.p_obj = ax.plot_surface(self.length * x + self.center_x,
                                     self.width * y + self.center_y,
                                     self.height * z + self.center_z,
                                     alpha=0.08,
                                     shade=False,
                                     )

    def rm_plot(self):
        if self.p_obj:
            self.p_obj.remove()
        self.p_obj = None

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

    def plot(self, color, ax):
        if self.dim == 2:
            self.p_obj = ax.scatter(self.tuple_identifier[0], self.tuple_identifier[1], c=color, s=10, edgecolor='none')
        elif self.dim == 3:
            self.p_obj = ax.scatter(self.tuple_identifier[0], self.tuple_identifier[1], self.tuple_identifier[2], c=color, s=10, edgecolor='none')

    def rm_plot(self):
        self.p_obj.remove()

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

















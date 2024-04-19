import math
import numpy as np
from utils.rtree.rstar_tree import RTree

###############################################################################
# Helper Functions                                                            #
###############################################################################


def get_angle(v0, v1):
    cos_theta = np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1))

    if cos_theta > 1:
        cos_theta = 1

    elif cos_theta < -1:
        cos_theta = -1

    return np.arccos(cos_theta)


###############################################################################
# Map Object                                                                  #
###############################################################################


class Map():

    def __init__(self, obstacles, dim=2):
        self.obstacles = obstacles
        self.dim = dim
        self.obs_tree = RTree(10, dim=self.dim)

        for o in self.obstacles:
            for f in o.faces:
                self.obs_tree.Insert(f)

    def sample_init(self, scope):
        self.overlay = scope

    def sample(self):
        return self.overlay.sample()

    def add_path(self, path):
        self.path = path

    def intersects_line(self, line):
        return

    def intersections(self, line):
        return

    def plot_path(self, path):
        return

    def in_free_space(self, point):
        for o in self.obstacles:
            if o.contains_point(point):
                return False

        return True

    def reset(self):
        if self.dim == 2:
            self.sample_init(SampleScope.Rectangle([400, 400], 600, 600))

        elif self.dim == 3:
            self.sample_init(SampleScope.Cube([400, 400, 400], 600, 600, 600))


class SampleScope():

    class Rectangle():
        def __init__(self, center, xoff, yoff):
            self.min_x = center[0] - xoff
            self.max_x = center[0] + xoff
            self.min_y = center[1] - yoff
            self.max_y = center[1] + yoff
            self.volume = (self.max_x - self.min_x) * (self.max_y - self.min_y)

        def sample(self):
            rand_x = (self.max_x - self.min_x) * np.random.random_sample() + self.min_x
            rand_y = (self.max_y - self.min_y) * np.random.random_sample() + self.min_y
            return np.array([rand_x, rand_y])

    class Cube():
        def __init__(self, center, xoff, yoff, zoff):
            self.min_x = center[0] - xoff
            self.max_x = center[0] + xoff
            self.min_y = center[1] - yoff
            self.max_y = center[1] + yoff
            self.min_z = center[2] - zoff
            self.max_z = center[2] + zoff
            self.volume = (self.max_x - self.min_x) * (self.max_y - self.min_y) * (self.max_z - self.min_z)

        def sample(self):
            rand_x = (self.max_x - self.min_x) * np.random.random_sample() + self.min_x
            rand_y = (self.max_y - self.min_y) * np.random.random_sample() + self.min_y
            rand_z = (self.max_z - self.min_z) * np.random.random_sample() + self.min_z
            return np.array([rand_x, rand_y, rand_z])

    class Ellipse(object):
        def __init__(self, f1, f0, d):
            self.f0 = f0
            self.f1 = f1

            diff_vec = f1 - f0

            self.center = 0.5 * diff_vec + f0
            self.a = d / 2
            self.c = 0.5 * np.linalg.norm(diff_vec)
            self.b = math.sqrt(pow(self.a, 2) - pow(self.c, 2))

            x_vec = np.array([1, 0])
            self.angle = get_angle(x_vec, diff_vec)
            cos_theta = math.cos(self.angle)
            sin_theta = math.sin(self.angle)
            self.rotate = np.array([[cos_theta, -sin_theta],
                                    [sin_theta, cos_theta]])

        # Sample from and ellipse with foci at p0, p1, defined by distance d
        def sample(self, buffer=1):
            center = self.center

            r1 = np.random.random_sample()
            r2 = np.random.random_sample()
            x_rand = buffer * self.a * (math.sqrt(r1) + r1) / 2
            y_rand = buffer * self.b * (math.sqrt(r2) + r2) / 2
            theta = np.random.random_sample() * 2 * math.pi
            p = np.array([x_rand * math.cos(theta),
                          y_rand * math.sin(theta)])

            p = np.dot(self.rotate, p)
            return p + center

    class Spheroid(object):
        def __init__(self, f1, f0, d):
            self.f0 = f0
            self.f1 = f1

            diff_vec = f1 - f0

            self.center = 0.5 * diff_vec + f0
            self.a = d / 2
            self.c = 0.5 * np.linalg.norm(diff_vec)
            self.b = math.sqrt(pow(self.a, 2) - pow(self.c, 2))

            rot_x = diff_vec / np.linalg.norm(diff_vec)
            rot_z = np.cross(np.array([1, 0, 0]), diff_vec)
            rot_z = rot_z / np.linalg.norm(rot_z)
            rot_y = np.cross(rot_z, rot_x)
            rot_y = rot_y / np.linalg.norm(rot_y)
            self.rotate = np.c_[rot_x, rot_y, rot_z]

        def sample(self, buffer=1.2):
            center = self.center

            r1 = np.random.random_sample()
            r2 = np.random.random_sample()
            r3 = np.random.random_sample()

            x_rand = buffer * self.a * r1 ** (1/3)
            y_rand = buffer * self.b * r2 ** (1/3)
            z_rand = buffer * self.b * r3 ** (1/3)

            theta = np.random.random_sample() * 2 * math.pi
            phi = np.random.random_sample() * math.pi

            p = np.array([x_rand * math.cos(phi) * math.sin(theta),
                          y_rand * math.sin(phi) * math.sin(theta),
                          z_rand * math.cos(theta)])

            p = np.dot(self.rotate, p)
            return p + center

    class ObliqueRectangle(object):
        def __init__(self, p0, p1, p_base):
            return

    class ObliqueCylinder(object):
        def __init__(self, p0, p1, p_base0):
            self.p0 = p0
            self.v = p1 - p0
            p_base1 = np.cross(p1 - p0, p_base0)
            p_base1 = p_base1 / np.linalg.norm(p_base1)
            self.normal = np.cross(p_base0, p_base1)
            self.base = np.linalg.norm(p_base0)

            rot_z = self.normal / np.linalg.norm(self.normal)
            rot_x = np.cross(rot_z, self.v)
            rot_x = rot_x / np.linalg.norm(rot_x)
            rot_y = np.cross(rot_z, rot_x)
            rot_y = rot_y / np.linalg.norm(rot_y)

            self.rotate = np.c_[rot_x, rot_y, rot_z]

        def sample(self):
            r0 = np.random.random_sample()
            r1 = np.random.random_sample()
            r2 = np.random.random_sample()

            theta = np.random.random_sample() * 2 * np.pi
            x = self.base * np.sqrt(r0) * np.cos(theta)
            y = self.base * np.sqrt(r1) * np.sin(theta)
            z = r2 * self.v

            p = np.array([x, y, 0])
            p = np.dot(self.rotate, p)
            p = p + z

            return p + self.p0

    class EpRegion(object):
        def __init__(self, path, dist):
            self.extension = []
            self.bag = []
            self.path_length = 0
            self.segments = []
            self.num_nodes = len(path)

            if path:
                start = path[0]
                dim = len(start)

                for i in range(len(path) - 2):
                    p0 = path[i]
                    p1 = path[i + 1]
                    p2 = path[i + 2]

                    v0 = p0 - p1
                    v0_norm = np.linalg.norm(v0)
                    v1 = p2 - p1
                    v0 = v0 / v0_norm
                    v1 = v1 / np.linalg.norm(v1)

                    self.path_length += v0_norm
                    self.segments.append(v0_norm)

                    v_mid = v0 + v1
                    v_mid = dist * v_mid / np.linalg.norm(v_mid)

                    if dim == 2:
                        self.extension.append(SampleScope.ObliqueRectangle(p0, p1, v_mid))

                    if dim == 3:
                        self.extension.append(SampleScope.ObliqueCylinder(p0, p1, v_mid))

                if dim == 2:
                    return

                elif dim == 3:
                    p0 = path[-2]
                    p1 = path[-1]
                    p2 = 2 * p1 - p0
                    rot = np.array([[0, -1, 0],
                                    [1, 0, 0],
                                    [0, 0, 1]])

                    v_mid = np.dot(rot, (p1 - p0))
                    v_mid = dist * v_mid / np.linalg.norm(v_mid)

                    l_norm = np.linalg.norm(path[len(path) - 2] - path[len(path) - 1])
                    self.path_length += l_norm
                    self.segments.append(l_norm)

                    self.extension.append(SampleScope.ObliqueCylinder(p0, p1, v_mid))

            for i in range(len(self.segments)):
                p = math.floor(self.segments[i] / self.path_length * 100)
                for _ in range(p):
                    self.bag.append(i)

        def sample(self):
            s = self.extension[np.random.choice(self.bag)].sample()

            return s





















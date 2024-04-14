import numpy as np
import math


###############################################################################
# Map Object                                                                  #
###############################################################################


class Map():

    def __init__(self, obstacles, dim=2):
        self.obstacles = obstacles
        self.dim = dim

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


class SampleScope():

    class Rectangle():
        def __init__(self, center, xoff, yoff):
            self.min_x = center[0] - xoff
            self.max_x = center[0] + xoff
            self.min_y = center[1] - yoff
            self.max_y = center[1] + yoff

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
            cos_theta = np.dot(x_vec, diff_vec) / (np.linalg.norm(x_vec) * np.linalg.norm(diff_vec))
            if cos_theta > 1:
                cos_theta = 1
            elif cos_theta < -1:
                cos_theta = -1

            self.angle = np.arccos(cos_theta)

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

            cos_theta = math.cos(self.angle)
            sin_theta = math.sin(self.angle)
            rotate = np.array([[cos_theta, -sin_theta],
                              [sin_theta, cos_theta]])

            p = np.dot(rotate, p)
            return np.array([p[0] + center[0], p[1] + center[1]])

    class Spheroid(object):
        def __init__(self, f1, f0, d):
            self.f0 = f0
            self.f1 = f1

            diff_vec = f1 - f0

            self.center = 0.5 * diff_vec + f0
            self.a = d / 2
            self.c = 0.5 * np.linalg.norm(diff_vec)
            self.b = math.sqrt(pow(self.a, 2) - pow(self.c, 2))

            x_vec = np.array([1, 0])
            p_z_vec = diff_vec[:2]
            cos_phi = np.dot(x_vec, p_z_vec) / np.linalg.norm(p_z_vec)

            if cos_phi > 1:
                cos_phi = 1
            elif cos_phi < -1:
                cos_phi = -1

            self.phi = np.arccos(cos_phi)
            sin_phi = np.sin(self.phi)

            y_vec = np.array([1, 0])
            p_x_vec = diff_vec[1:]
            cos_theta = np.dot(y_vec, p_x_vec) / np.linalg.norm(p_x_vec)

            if cos_theta > 1:
                cos_theta = 1
            elif cos_theta < -1:
                cos_theta = -1

            self.theta = -np.arccos(cos_theta)
            sin_theta = np.sin(self.theta)
            rot_x = np.array([cos_phi, sin_phi, -sin_theta])
            rot_y = np.array([-sin_phi, cos_phi, -sin_theta])
            rot_z = np.array([0, -sin_phi, cos_phi])
            rot_x = rot_x / np.linalg.norm(rot_x)
            rot_y = rot_y / np.linalg.norm(rot_y)
            rot_z = rot_z / np.linalg.norm(rot_z)
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
            return np.array([p[0] + center[0],
                             p[1] + center[1],
                             p[2] + center[2]
                             ])





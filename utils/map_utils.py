import numpy as np
import math
from rrt_methods.rrt_utils import ray_cast
from rrt_methods.rrt_utils import sample_free
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


###############################################################################
# Map Object                                                                  #
###############################################################################


class Map():

    def __init__(self, region, obstacles, dim=2):
        self.region = region
        self.obstacles = obstacles
        self.dim = dim
        self.ax = None

    def plot(self):

        if self.dim == 2:
            _, self.ax = plt.subplots()

        elif self.dim == 3:

            f = plt.figure()
            ax = f.add_subplot(1, 1, 1, projection=Axes3D.name)
            self.ax = ax

    def sample_init(self, scope):
        self.overlay = scope

    def sample(self):
        if type(self.overlay) is Rectangle:
            return sample_free(self.overlay, self.region, self.obstacles)

        if type(self.overlay) is Ellipse:
            return self.overlay.sample()

        if type(self.overlay) is Cube:
            return self.overlay.sample()

    def add_path(self, path):
        self.path = path

    def plot_path(self, path):
        return

    # For 3d plotting of the tree
    def animate(self):

        if self.dim == 3 and self.ax:
            for angle in range(0, 1000, 2):

                self.ax.view_init(elev=angle + math.sin(1 / (angle + 1)) / 5, azim=.7 * angle, roll=.8 * angle)
                plt.draw()
                plt.pause(.001)

            plt.show()


class Rectangle(object):
    def __init__(self, bounds):
        self.min_x = bounds[0]
        self.max_x = bounds[1]
        self.min_y = bounds[2]
        self.max_y = bounds[3]
        self.x = [self.min_x, self.min_x, self.max_x, self.max_x, self.min_x]
        self.y = [self.min_y, self.max_y, self.max_y, self.min_y, self.min_y]
        self.vertices = [np.array([self.x[i], self.y[i]]) for i in range(4)]

    def sample(self):
        rand_x = (self.max_x - self.min_x) * np.random.random_sample() + self.min_x
        rand_y = (self.max_y - self.min_y) * np.random.random_sample() + self.min_y
        return np.array([rand_x, rand_y])

    def contains_point(self, p):
        return ray_cast(self.vertices, p)

    def plot(self, ax):
        ax.plot(self.x, self.y, linewidth=.5)


class Cube(object):
    def __init__(self, bounds):
        self.min_x = bounds[0]
        self.max_x = bounds[1]
        self.min_y = bounds[2]
        self.max_y = bounds[3]
        self.min_z = bounds[4]
        self.max_z = bounds[5]

        self.length = self.max_x - self.min_x
        self.width = self.max_y - self.min_y
        self.height = self.max_z - self.min_z

        self.center_x = self.min_x + self.length / 2
        self.center_y = self.min_y + self.width / 2
        self.center_z = self.min_z + self.height / 2
        self.center = np.array([self.center_x, self.center_y, self.center_z])

    def sample(self):
        rand_x = (self.max_x - self.min_x) * np.random.random_sample() + self.min_x
        rand_y = (self.max_y - self.min_y) * np.random.random_sample() + self.min_y
        rand_z = (self.max_z - self.min_z) * np.random.random_sample() + self.min_z
        return np.array([rand_x, rand_y, rand_z])

    def contains_point(self, p):
        x = self.min_x <= p[0] <= self.max_x
        y = self.min_y <= p[1] <= self.may_y
        z = self.min_z <= p[2] <= self.maz_z
        return x and y and z

    def get_cube():

        phi = np.arange(1, 10, 2) * np.pi / 4
        Phi, Theta = np.meshgrid(phi, phi)

        x = np.cos(Phi) * np.sin(Theta)
        y = np.sin(Phi) * np.sin(Theta)
        z = np.cos(Theta) / np.sqrt(2)

        return x, y, z

    def plot(self, ax):
        x, y, z = Cube.get_cube()
        ax.plot_surface(self.length * x + self.center[0],
                        self.width * y + self.center[1],
                        self.height * z + self.center[2],
                        alpha=0.08,
                        shade=False,
                        )


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

        cos_theta = math.cos(-self.angle)
        sin_theta = math.sin(-self.angle)
        rotate = np.array([[cos_theta, -sin_theta],
                          [sin_theta, cos_theta]])

        p = np.dot(rotate, p)
        return np.array([p[0] + center[0], p[1] + center[1]])


###############################################################################
# Plotting                                                                    #
###############################################################################


def plot_path(path, c="#000000", ax=None):
    path_x = []
    path_y = []
    for i in range(len(path)):
        idx = i % len(path)
        path_x.append(path[idx][0])
        path_y.append(path[idx][1])
    ax.plot(path_x, path_y, c=c)


def plot_poly(path, c="#000000", ax=None):
    path_x = []
    path_y = []
    for i in range(len(path) + 1):
        idx = i % len(path)
        path_x.append(path[idx][0])
        path_y.append(path[idx][1])
    ax.plot(path_x, path_y, c=c)






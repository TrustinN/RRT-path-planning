import math
import numpy as np
from random import randrange
from .make_race import new_race
from .map_utils import plot_poly
from .map_utils import ray_cast
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rrt_methods.rrt_utils import sample_free
from rrt_methods.rrt_utils import sample_ellipse
from r_trees.r_tree_utils import Rect
from r_trees.r_tree_utils import Cube
from r_trees.r_tree_utils import IndexRecord

###############################################################################
# Sampling random point                                                       #
###############################################################################


class Sample_Scope():

    def __init__(self, name, scope, map=None):
        self.name = name
        self.overlay = scope


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

    def sample_init(self, name, scope):
        self.scope = Sample_Scope(name, scope)

    def add_path(self, path):
        self.path = path


# Need to create a separate sample space object
class Sampler(object):

    def __init__(self, map):
        self.scope = map.scope.name
        self.region = map.region
        self.obstacles = map.obstacles
        self.overlay = map.scope.overlay

    def sample(self):
        if self.scope == "box":
            return sample_free(self.overlay, self.region, self.obstacles)
        if self.scope == "ellipse":
            return sample_ellipse(self.overlay, buffer=1)


class AxMin():

    def __init__(self):
        self.min = math.inf

    def compare(self, num):
        if num < self.min:
            self.min = num


class AxMax():

    def __init__(self):
        self.max = -math.inf

    def compare(self, num):
        if num > self.max:
            self.max = num


class Obstacle(IndexRecord):

    class Facet():

        def __init__(self, vertices, dim):

            self.dim = dim
            self.vertices = vertices
            self.num_vertices = len(self.vertices)
            bounds = []

            for i in range(dim):
                bounds.append(AxMin())
                bounds.append(AxMax())

            for i in range(self.num_vertices):
                curr_vertex = vertices[i]
                for j in range(dim):
                    bounds[2 * j].compare(curr_vertex[j])
                    bounds[2 * j + 1].compare(curr_vertex[j])

            self.bound = []
            for i in range(dim):
                self.bound.append(bounds[2 * i].min)
                self.bound.append(bounds[2 * i + 1].max)

            if dim == 2:
                self.bound = Rect(self.bound)

            elif dim == 3:
                self.bound = Cube(self.bound)

        def plot(self, ax):
            ax.add_collection3d(Poly3DCollection([self.vertices], alpha=0.08))
            for v in self.vertices:
                ax.scatter(v[0], v[1], v[2], s=10, edgecolor='none')

        def plot_bound(self, ax):
            self.bound.plot(10, ax)

    def __init__(self, faces, dim=3):
        self.faces = faces
        self.bounds = [f.bound for f in faces]

        if dim == 3:
            self.bound = Cube.combine(self.bounds)

    def plot(self, ax=None):
        for f in self.faces:
            f.plot(ax=ax)

    def plot_bound(self, ax):
        self.bound.plot(10, ax)

    def animate(self, ax):

        for angle in range(0, 1000, 1):

            ax.view_init(elev=angle + math.sin(1 / (angle + 1)) / 5, azim=.7 * angle, roll=.8 * angle)
            plt.draw()
            plt.pause(.001)

        plt.show()


def make_rect_region(min_x, max_x, max_y, min_y):
    b_l = np.array([min_x, min_y])
    b_r = np.array([max_x, min_y])
    t_l = np.array([min_x, max_y])
    t_r = np.array([max_x, max_y])
    return [b_l, b_r, t_r, t_l]


# Hacky obstacle maker (rectangle)
def obstacle_to_path(p0_r, p0_l, p1_r, p1_l):
    obstacle = []
    dir_r = p1_r - p0_r
    dir_l = p1_l - p0_l
    obstacle.append(.1 * dir_r + p0_r)
    obstacle.append(.1 * dir_l + p0_l)
    return obstacle


# makes rectangle with 'foci' p0, p1
def expand_obstacle(p0, p1, r):
    t_l = np.array([p0[0], p0[1] + r])
    b_l = np.array([p0[0], p0[1] - r])
    b_r = np.array([p1[0], p1[1] - r])
    t_r = np.array([p1[0], p1[1] + r])

    return [t_l, b_l, b_r, t_r]


class RaceMap(Map):

    def __init__(self, racetrack=None, n=20):

        SCREEN_WIDTH = 800
        SCREEN_HEIGHT = 800
        POINT_RADIUS = 5
        POINT_COLOR = (255, 0, 0)
        LINE_COLOR = (0, 0, 255)
        ROAD_THICKNESS = 50
        screen = None
        if not racetrack:
            racetrack = new_race(SCREEN_WIDTH,
                                 SCREEN_HEIGHT,
                                 POINT_RADIUS,
                                 screen,
                                 POINT_COLOR,
                                 LINE_COLOR,
                                 ROAD_THICKNESS,
                                 randrange(1000))
        midpoints = racetrack.generate_race_course_midpath(n)
        blue_cones, yellow_cones = racetrack.generate_left_and_right_cones()
        self.blue_cones = blue_cones
        self.yellow_cones = yellow_cones

        start_pos = ((blue_cones[-1] - yellow_cones[-1]) / 2) + yellow_cones[-1]
        end_pos = ((blue_cones[-2] - yellow_cones[-2]) / 2) + yellow_cones[-2]

        obs1 = obstacle_to_path(
                blue_cones[-1],
                yellow_cones[-1],
                blue_cones[-2],
                yellow_cones[-2]
                )

        obs2 = obstacle_to_path(
                blue_cones[-2],
                yellow_cones[-2],
                blue_cones[-1],
                yellow_cones[-1]
                )

        obstacles = [blue_cones, obs1, obs2]
        super().__init__(region=yellow_cones, obstacles=obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def plot(self):

        super().plot()
        plot_poly(self.blue_cones, c="#e13c41", ax=self.ax)
        plot_poly(self.yellow_cones, c="#e13c41", ax=self.ax)


class SquareObsMap(Map):
    def __init__(self, n, size):

        start_pos = np.array([10, 10])
        end_pos = np.array([790, 790])
        min_x, max_x, max_y, min_y = -200, 1000, 1000, -200
        b_l = np.array([min_x, min_y])
        b_r = np.array([max_x, min_y])
        t_l = np.array([min_x, max_y])
        t_r = np.array([max_x, max_y])
        region = [b_l, b_r, t_r, t_l]

        obstacles = []
        while n > 0:
            x_rand = max_x * np.random.random_sample()
            y_rand = max_y * np.random.random_sample()
            b_l = np.array([x_rand - size, y_rand - size])
            b_r = np.array([x_rand + size, y_rand - size])
            t_l = np.array([x_rand - size, y_rand + size])
            t_r = np.array([x_rand + size, y_rand + size])
            o = [b_l, b_r, t_r, t_l]
            if not ray_cast(o, start_pos) and not ray_cast(o, end_pos):
                obstacles.append(o)
                n -= 1

        super().__init__(region=region, obstacles=obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def plot(self):
        super().plot()
        plot_poly(self.region, c="#e13c41", ax=self.ax)
        for o in self.obstacles:
            plot_poly(o, c="#e13c41", ax=self.ax)


# Grid is the number of divisions of our maze we want from
# the starting square
class Maze(Map):
    def __init__(self, grid):
        min_x, max_x, max_y, min_y = 0, 800, 800, 0
        d = max(max_x, max_y)
        box_length = math.floor(d // grid)
        mid = box_length / 2
        start_pos = np.array([mid, mid])
        end_pos = np.array([d - mid, d - mid])
        obstacles = []

        region = make_rect_region(min_x, max_x, max_y, min_y)
        for i in range(grid - 1):
            obs_pos = []
            new_v = 0
            while True:
                new_v = np.random.randint(new_v + 1, high=new_v + 5)
                obs_pos.append(new_v)
                if new_v >= grid - 1:
                    break
            obstacles.append(obs_pos)

        return_obstacles = []
        for i in range(len(obstacles)):
            for j in range(len(obstacles[i]) - 1):
                c0 = obstacles[i][j]
                c1 = c0 + 1
                p0 = np.array([c0 * box_length, (i + 1) * box_length])
                p1 = np.array([c1 * box_length, (i + 1) * box_length])
                rect_obs = expand_obstacle(p0, p1, 10)
                return_obstacles.append(rect_obs)

        super().__init__(region=region, obstacles=return_obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def plot(self):
        super().plot()
        plot_poly(self.region, c="#e13c41", ax=self.ax)
        for o in self.obstacles:
            plot_poly(o, c="#e13c41", ax=self.ax)





















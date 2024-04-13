import math
import numpy as np
# from random import randrange
# from .make_race import new_race
from .map_utils import Map
from .map_utils import Rectangle
from utils.rtree.rstar_tree import RTree
from utils.rtree.rtree_utils import Rect as RectBound
from utils.quickhull.hull import QuickHull


# # Hacky obstacle maker (rectangle)
# def obstacle_to_path(p0_r, p0_l, p1_r, p1_l):
#     obstacle = []
#     dir_r = p1_r - p0_r
#     dir_l = p1_l - p0_l
#     obstacle.append(.1 * dir_r + p0_r)
#     obstacle.append(.1 * dir_l + p0_l)
#     return obstacle


# class RaceMap(Map):
#
#     def __init__(self, racetrack=None, n=20):
#
#         SCREEN_WIDTH = 800
#         SCREEN_HEIGHT = 800
#         POINT_RADIUS = 5
#         POINT_COLOR = (255, 0, 0)
#         LINE_COLOR = (0, 0, 255)
#         ROAD_THICKNESS = 50
#         screen = None
#         if not racetrack:
#             racetrack = new_race(SCREEN_WIDTH,
#                                  SCREEN_HEIGHT,
#                                  POINT_RADIUS,
#                                  screen,
#                                  POINT_COLOR,
#                                  LINE_COLOR,
#                                  ROAD_THICKNESS,
#                                  randrange(1000))
#         midpoints = racetrack.generate_race_course_midpath(n)
#         blue_cones, yellow_cones = racetrack.generate_left_and_right_cones()
#         self.blue_cones = blue_cones
#         self.yellow_cones = yellow_cones
#
#         start_pos = ((blue_cones[-1] - yellow_cones[-1]) / 2) + yellow_cones[-1]
#         end_pos = ((blue_cones[-2] - yellow_cones[-2]) / 2) + yellow_cones[-2]
#
#         obs1 = obstacle_to_path(
#                 blue_cones[-1],
#                 yellow_cones[-1],
#                 blue_cones[-2],
#                 yellow_cones[-2]
#                 )
#
#         obs2 = obstacle_to_path(
#                 blue_cones[-2],
#                 yellow_cones[-2],
#                 blue_cones[-1],
#                 yellow_cones[-1]
#                 )
#
#         obstacles = [blue_cones, obs1, obs2]
#         super().__init__(region=yellow_cones, obstacles=obstacles, dim=2)
#         self.add_path([start_pos, end_pos])
#
#     def plot(self):
#
#         super().plot()
#         plot_poly(self.blue_cones, c="#e13c41", ax=self.ax)
#         plot_poly(self.yellow_cones, c="#e13c41", ax=self.ax)

class RaceMap():
    def __init__(self):
        return


class RandomObsMap(Map):
    def __init__(self, n, size):

        start_pos = np.array([10, 10])
        end_pos = np.array([790, 790])
        bounds = [(-200, 1000), (-200, 1000)]
        region = Rectangle(bounds)

        self.obs_tree = RTree(10, dim=2)
        obstacles = []
        bounds = [100, 700, 100, 700]
        while n > 0:
            x_rand = (bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0]
            y_rand = (bounds[3] - bounds[2]) * np.random.random_sample() + bounds[2]

            center = [x_rand, y_rand]
            c = [(center[i] - size, center[i] + size) for i in range(2)]
            cube = Rectangle(c)

            o = []
            for i in range(10):
                o.append(cube.sample())
            o = QuickHull(o)

            if not o.contains_point(start_pos) and not o.contains_point(end_pos):
                obstacles.append(o)
                for f in o.faces:
                    self.obs_tree.Insert(f)
                n -= 1

        super().__init__(region=region, obstacles=obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def intersects_line(self, line):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        # Return true if line segments AB and CD intersect
        def intersect(A, B, C, D):
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

        start = line[0]
        end = line[1]
        l_bound = RectBound([min(start[0], end[0]), max(start[0], end[0]), min(start[1], end[1]), max(start[1], end[1])])
        potential = self.obs_tree.SearchOverlap(l_bound)
        for f in potential:
            if intersect(start, end, f.vertices[0], f.vertices[1]):
                return True

        return False

    def plot(self, view):
        for hull in self.obstacles:
            hull.plot(view)


# Grid is the number of divisions of our maze we want from
# the starting square
class Maze(Map):
    def __init__(self, grid):
        bounds = [-200, 1000, -200, 1000]
        d = max(bounds[1], bounds[3])

        box_length = math.floor(d // grid)
        mid = box_length / 2

        start_pos = np.array([mid, mid])
        end_pos = np.array([d - mid, d - mid])

        obstacles = []
        region = Rectangle(bounds)

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
                min_x = c0 * box_length
                max_x = c1 * box_length
                min_y = (i + 1) * box_length - 10
                max_y = (i + 1) * box_length + 10
                bound = [min_x, max_x, min_y, max_y]
                rect_obs = Rectangle(bound)
                return_obstacles.append(rect_obs)

        super().__init__(region=region, obstacles=return_obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def plot(self):
        super().plot()
        self.region.plot(self.ax)

        for o in self.obstacles:
            o.plot(self.ax)





















import math
import numpy as np
from .map_utils import Map
from .map_utils import SampleScope
from utils.rtree.rstar_tree import RTree
from utils.rtree.rtree_utils import Rect
from utils.quickhull.hull import QuickHull


class RandomObsMap(Map):
    def __init__(self, n, size):

        start_pos = np.array([10, 10])
        end_pos = np.array([790, 790])

        self.obs_tree = RTree(10, dim=2)
        ospace = SampleScope.Rectangle([400, 400], 300, 300)
        obstacles = []

        while n > 0:
            center = ospace.sample()
            r = SampleScope.Rectangle(center, size, size)

            o = []
            for i in range(10):
                o.append(r.sample())
            o = QuickHull(o)

            if not o.contains_point(start_pos) and not o.contains_point(end_pos):
                obstacles.append(o)
                for f in o.faces:
                    self.obs_tree.Insert(f)
                n -= 1

        super().__init__(obstacles=obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def intersects_line(self, line):

        start = line[0]
        end = line[1]
        l_bound = Rect.combine_points([start, end])
        potential = self.obs_tree.SearchOverlap(l_bound)

        for f in potential:
            if f.intersects_line(start, end):
                return True

        return False

    def intersections(self, line):
        ints = []
        l_bound = Rect.combine_points(line)
        potential = self.obs_tree.SearchOverlap(l_bound)

        for o in potential:
            inter = o.intersections(line)

            if type(inter) is np.ndarray:
                ints.append(inter)

        return ints

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
        region = SampleScope.Rectangle(bounds)

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
                rect_obs = SampleScope.Rectangle(bound)
                return_obstacles.append(rect_obs)

        super().__init__(region=region, obstacles=return_obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def plot(self):
        super().plot()
        self.region.plot(self.ax)

        for o in self.obstacles:
            o.plot(self.ax)





















import numpy as np
from .map_utils import Map
from .map_utils import SampleScope
from utils.rtree.rtree_utils import Rect
from utils.quickhull.hull import QuickHull
import pyqtgraph as pg


class Map2d(Map):
    def __init__(self, obstacles, dim):
        super().__init__(obstacles, dim)

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
        for o in self.obstacles:
            o.plot(view)

    def plot_path(self, path, view):
        for i in range(len(path) - 1):
            line = pg.PlotDataItem(np.array([path[i], path[i + 1]]),
                                   connect="all",
                                   pen=pg.mkPen("#ff00ff"),)
            view.addItem(line)


class RandomObsMap(Map2d):
    def __init__(self, n, size):

        start_pos = np.array([10, 10])
        end_pos = np.array([790, 790])

        ospace = SampleScope.Rectangle([400, 400], 500, 500)
        obstacles = []

        while n > 0:
            center = ospace.sample()
            r = SampleScope.Rectangle(center, size, size)

            o = QuickHull([r.sample() for i in range(10)])

            if not o.contains_point(start_pos) and not o.contains_point(end_pos):
                obstacles.append(o)
                n -= 1

        super().__init__(obstacles=obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def reset(self):
        self.sample_init(SampleScope.Rectangle([400, 400], 600, 600))

















import numpy as np
from .map_utils import Map
from utils.quickhull.hull import QuickHull
from utils.rtree.rstar_tree import RTree
from utils.maps.map_utils import SampleScope
from utils.rtree.rtree_utils import Cube


class RandomObsMap(Map):
    def __init__(self, n, size):

        start_pos = np.array([10, 10, 10])
        end_pos = np.array([790, 790, 790])

        self.obs_tree = RTree(10, dim=3)
        obstacles = []
        ospace = SampleScope.Cube([400, 400, 400], 300, 300, 300)
        while n > 0:
            center = ospace.sample()
            cube = SampleScope.Cube(center, size, size, size)

            o = []
            for i in range(10):
                o.append(cube.sample())
            o = QuickHull(o)

            if not o.contains_point(start_pos) and not o.contains_point(start_pos):
                obstacles.append(o)
                for f in o.faces:
                    self.obs_tree.Insert(f)
                n -= 1

        super().__init__(obstacles=obstacles, dim=3)
        self.add_path([start_pos, end_pos])

    def intersects_line(self, line):

        start = line[0]
        end = line[1]
        l_bound = Cube.combine_points([start, end])
        potential = self.obs_tree.SearchOverlap(l_bound)
        for f in potential:
            if f.intersects_line(start, end):
                return True

        return False

    def intersections(self, line):
        ints = []
        l_bound = Cube.combine_points(line)
        potential = self.obs_tree.SearchOverlap(l_bound)

        for o in potential:
            inter = o.intersections(line)

            if type(inter) is np.ndarray:
                ints.append(inter)

        return ints

    def plot(self, view):
        for hull in self.obstacles:
            hull.plot(view)




















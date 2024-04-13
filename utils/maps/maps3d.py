import numpy as np
from .map_utils import Map
from .map_utils import Cube
from utils.quickhull.hull import QuickHull
from utils.rtree.rstar_tree import RTree
from utils.rtree.rtree_utils import Cube as CubeBound


class RandObsMap(Map):
    def __init__(self, n, size):

        start_pos = np.array([10, 10, 10])
        end_pos = np.array([790, 790, 790])
        bounds = [-200, 1000, -200, 1000, -200, 1000]
        vertices = [np.array([bounds[i], bounds[j + 2], bounds[k + 4]]) for i in range(2) for j in range(2) for k in range(2)]
        region = QuickHull(vertices)

        self.obs_tree = RTree(10, dim=3)
        obstacles = []
        bounds = [100, 700, 100, 700, 100, 700]
        while n > 0:
            x_rand = (bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0]
            y_rand = (bounds[3] - bounds[2]) * np.random.random_sample() + bounds[2]
            z_rand = (bounds[5] - bounds[4]) * np.random.random_sample() + bounds[4]

            center = [x_rand, y_rand, z_rand]
            c = [(center[i] - size, center[i] + size) for i in range(3)]
            cube = Cube(c)

            o = []
            for i in range(10):
                o.append(cube.sample())
            o = QuickHull(o)

            if not o.contains_point(start_pos) and not o.contains_point(start_pos):
                obstacles.append(o)
                for f in o.faces:
                    self.obs_tree.Insert(f)
                n -= 1

        super().__init__(region=region, obstacles=obstacles, dim=3)
        self.add_path([start_pos, end_pos])

    def intersects_line(self, line):
        if self.region.intersects_line(line):
            return True

        start = line[0]
        end = line[1]
        vec = end - start
        l_bound = CubeBound([min(start[0], end[0]), max(start[0], end[0]), min(start[1], end[1]), max(start[1], end[1]), min(start[2], end[2]), max(start[2], end[2])])
        potential = self.obs_tree.SearchOverlap(l_bound)
        for f in potential:
            if l_bound.overlap(f.bound) > 0:
                if np.sign(f.orient(start)) != np.sign(f.orient(end)):

                    d = np.dot(f.normal, f.vertices[0])
                    na = np.dot(f.normal, start)
                    nv = np.dot(f.normal, vec)
                    t = (d - na) / nv
                    if 0 <= t <= 1:

                        inter = t * vec + start
                        iter = 0
                        if f.bound.contains_point(inter):
                            while iter < 3:
                                if f.neighbors[iter].orient(inter) < 0:
                                    break
                                iter += 1

                            if iter == 3:
                                return True

        return False

    def intersections(self, line):
        ints = self.region.intersections(line)
        for o in self.obstacles:
            ints += o.intersections(line)
        return ints

    def plot(self, view):
        for hull in self.obstacles:
            hull.plot(view)




















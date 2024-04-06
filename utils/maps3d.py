import numpy as np
from .map_utils import Map
from .map_utils import Cube
from .quickhull.main import QuickHull


class RandObsMap(Map):
    def __init__(self, n, size):

        start_pos = np.array([10, 10, 10])
        end_pos = np.array([790, 790, 790])
        bounds = [-200, 1000, -200, 1000, -200, 1000]
        vertices = [np.array([bounds[i], bounds[j + 2], bounds[k + 4]]) for i in range(2) for j in range(2) for k in range(2)]
        region = QuickHull(vertices)

        obstacles = []
        while n > 0:
            x_rand = bounds[1] * np.random.random_sample()
            y_rand = bounds[3] * np.random.random_sample()
            z_rand = bounds[5] * np.random.random_sample()

            center = [x_rand, y_rand, z_rand]
            c = [center[i // 2] + size * (-1) ** (i + 1) for i in range(6)]
            cube = Cube(c)

            o = []
            for i in range(10):
                o.append(cube.sample())
            o = QuickHull(o)

            if not o.contains_point(start_pos) and not o.contains_point(start_pos):
                obstacles.append(o)
                n -= 1

        super().__init__(region=region, obstacles=obstacles, dim=3)
        self.add_path([start_pos, end_pos])

    def plot(self):
        super().plot()
        for o in self.obstacles:
            o.plot(ax=self.ax)

    def plot_path(self, path):
        path_x, path_y, path_z = [], [], []
        for i in range(len(path)):
            idx = i % len(path)
            path_x.append(path[idx][0])
            path_y.append(path[idx][1])
            path_z.append(path[idx][2])

        self.ax.plot(path_x, path_y, zs=path_z, c="#000000")


















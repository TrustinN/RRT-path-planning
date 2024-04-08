import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
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
                n -= 1

        super().__init__(region=region, obstacles=obstacles, dim=3)
        self.add_path([start_pos, end_pos])

    def intersections(self, line):
        ints = self.region.intersections(line)
        for o in self.obstacles:
            ints += o.intersections(line)
        return ints

    def plot(self):
        super().plot()
        for hull in self.obstacles:
            hull.plot(self.view)

    def plot_path(self, path):
        for i in range(len(path) - 1):
            line = gl.GLLinePlotItem(pos=np.array([path[i], path[i + 1]]),
                                     color=pg.mkColor("#ff00ff"),
                                     width=10)
            line.setGLOptions("opaque")
            self.view.addItem(line)




















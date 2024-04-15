import timeit
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from .rrt import rrt_run as rrt
from .rrt_connect import rrt_run as rrt_connect
from .rrt_star import rrt_run as rrt_star
from .rrt_star_connect import rrt_run as rrt_star_connect
from .multi_rrt_star_connect import rrt_run as multi_rrt_star_connect
from .informed_rrt_star import rrt_run as informed_rrt_star
from .quick_rrt_star import rrt_run as quick_rrt_star
from .informed_quick_rrt_star import rrt_run as informed_quick_rrt_star


class RRTSolver():
    def __init__(self):
        self.method = "rrt"
        self.methods = {
            "rrt": rrt,
            "rrt_connect": rrt_connect,
            "rrt_star": rrt_star,
            "rrt_star_connect": rrt_star_connect,
            "multi_rrt_star_connect": multi_rrt_star_connect,
            "informed_rrt_star": informed_rrt_star,
            "quick_rrt_star": quick_rrt_star,
            "informed_quick_rrt_star": informed_quick_rrt_star
        }
        self.show_leaves = False
        self.show_branches = False

    def set_map(self, map):
        self.map = map

    def set_max_iter(self, iter):
        self.max_iter = iter

    def set_step_size(self, size):
        self.step_size = size

    def set_method(self, method):
        self.method = method

    def run(self):
        self.rrt_run = self.methods[self.method]

        time_start = timeit.default_timer()
        path, t_start, t_end = self.rrt_run(map=self.map, step_size=self.step_size, max_iter=self.max_iter)
        time_stop = timeit.default_timer()

        self.time = time_stop - time_start

        self.path = path
        self.t_start = t_start
        self.t_end = t_end

    def leaf_toggle(self):
        self.show_leaves = not self.show_leaves

    def branch_toggle(self):
        self.show_branches = not self.show_branches

    def plot(self, view):

        self.view = view
        if self.map.dim == 2:

            self.map.plot(view)
            self.t_start.plot(view, self.show_branches, self.show_leaves)
            if self.t_end:
                self.t_end.plot(view, self.show_branches, self.show_leaves)

            for i in range(len(self.path) - 1):
                line = pg.PlotDataItem(np.array([self.path[i], self.path[i + 1]]),
                                       connect="all",
                                       pen=pg.mkPen("#ff00ff"),)
                view.addItem(line)

        elif self.map.dim == 3:

            g = gl.GLGridItem()
            g.translate(400, 400, -200)
            g.scale(100, 100, 100)
            self.view.addItem(g)

            self.map.plot(view)
            self.t_start.plot(view, self.show_branches, self.show_leaves)
            if self.t_end:
                self.t_end.plot(view, self.show_branches, self.show_leaves)

            for i in range(len(self.path) - 1):
                line = gl.GLLinePlotItem(pos=np.array([self.path[i], self.path[i + 1]]),
                                         color=pg.mkColor("#ff00ff"),
                                         width=3,)
                line.setGLOptions("opaque")
                view.addItem(line)

    def get_time(self):
        return "Time (" + self.method + f"): {self.time}"





























import timeit
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class RRTSolver():
    def __init__(self, map, step_size, max_iter, method="rrt_star"):

        if method == "rrt":
            from .rrt import rrt_run

        elif method == "rrt_connect":
            from .rrt_connect import rrt_run

        elif method == "rrt_star":
            from .rrt_star import rrt_run

        elif method == "rrt_star_connect":
            from .rrt_star_connect import rrt_run

        elif method == "informed_rrt_star":
            from .informed_rrt_star import rrt_run

        elif method == "quick_rrt_star":
            from .quick_rrt_star import rrt_run

        elif method == "informed_quick_rrt_star":
            from .informed_quick_rrt_star import rrt_run

        time_start = timeit.default_timer()
        path, t_start, t_end = rrt_run(map=map, step_size=step_size, max_iter=max_iter)
        time_stop = timeit.default_timer()

        self.time = time_stop - time_start
        self.method = method

        self.map = map
        self.path = path
        self.t_start = t_start
        self.t_end = t_end

    def plot(self, branches, leaves):

        if self.map.dim == 2:

            pg.mkQApp("Map")
            self.view = pg.plot().getViewBox()
            self.view.show()

            self.map.plot(self.view)
            self.t_start.plot(self.view, branches, leaves)
            if self.t_end:
                self.t_end.plot(self.view, branches, leaves)

            for i in range(len(self.path) - 1):
                line = pg.PlotDataItem(np.array([self.path[i], self.path[i + 1]]),
                                       connect="all",
                                       pen=pg.mkPen("#ff00ff"),)
                self.view.addItem(line)

        elif self.map.dim == 3:

            pg.mkQApp("Map")
            self.view = gl.GLViewWidget()
            self.view.setCameraPosition(distance=1500)
            self.view.pan(400, 400, 400)
            self.view.show()

            g = gl.GLGridItem()
            g.translate(400, 400, -200)
            g.scale(100, 100, 100)
            self.view.addItem(g)

            self.map.plot(self.view)
            self.t_start.plot(self.view, branches, leaves)
            if self.t_end:
                self.t_end.plot(self.view, branches, leaves)

            for i in range(len(self.path) - 1):
                line = gl.GLLinePlotItem(pos=np.array([self.path[i], self.path[i + 1]]),
                                         color=pg.mkColor("#ff00ff"),
                                         width=10)
                line.setGLOptions("opaque")
                self.view.addItem(line)

    def get_time(self):
        return "Time (" + self.method + f"): {self.time}"




























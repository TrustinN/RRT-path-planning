import numpy as np
from .plot import PlotObject
from utils.maps.MapBuilder import MapBuilder
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class PlotHandler():
    def __init__(self):
        self.plotter = PlotObject()
        self.builder = MapBuilder()

    def reconfig_builder(self, dim, seed, name, *args):
        self.builder.redefine_params(dim, seed, name, args)

    def switch_dim(self, dim):
        self.builder.redefine_params(dim,
                                     self.builder.seed,
                                     self.builder.name,
                                     self.builder.args)
        self.plotter.set_dim(dim)

    def draw(self):
        self.builder.map.plot(self.plotter.get_view())

    def change_map(self):
        self.plotter.clear()
        self.builder.new_map()

    def reset(self):
        self.plotter.clear()
        self.builder.map.reset()

    def get_view(self):
        return self.plotter.view

    def get_widget(self):
        return self.plotter.get_widget()

    def get_map(self):
        return self.builder.get_map()

    def plot_solution(self, solver, branches, leaves):
        view = self.get_view()
        map = self.get_map()

        if map.dim == 2:
            solver.t_start.plot(view, branches, leaves)

            if solver.t_end:
                solver.t_end.plot(view, branches, leaves)

            for i in range(len(solver.path) - 1):
                line = pg.PlotDataItem(np.array([solver.path[i], solver.path[i + 1]]),
                                       connect="all",
                                       pen=pg.mkPen("#ff00ff"),)
                view.addItem(line)

        elif map.dim == 3:

            solver.t_start.plot(view, branches, leaves)
            if solver.t_end:
                solver.t_end.plot(view, branches, leaves)

            for i in range(len(solver.path) - 1):
                line = gl.GLLinePlotItem(pos=np.array([solver.path[i], solver.path[i + 1]]),
                                         color=pg.mkColor("#ff00ff"),
                                         width=3,)
                line.setGLOptions("opaque")
                view.addItem(line)






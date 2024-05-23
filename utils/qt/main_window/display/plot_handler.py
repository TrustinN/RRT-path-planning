from .plot import PlotObject
from utils.maps.MapBuilder import MapBuilder
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class PlotHandler():
    def __init__(self):
        self.plotter = PlotObject()
        self.builder = MapBuilder()

        self.plots = {
            2: {},
            3: {},
        }
        self.t1Data = {}
        self.t2Data = {}

        self.t1_active = False
        self.t2_active = False

    def reconfig_builder(self, dim, seed, name, *args):
        self.builder.redefine_params(dim, seed, name, args)

    def switch_dim(self, dim):
        self.builder.redefine_params(dim,
                                     self.builder.seed,
                                     self.builder.name,
                                     self.builder.args)
        self.plotter.set_dim(dim)
        self.plot = self.plots[dim]

    def updateMD(self, dname, data, option='opaque'):
        contain = True
        if self.plotter.dim == 2:
            if not self.plot.__contains__(dname):

                self.plot[dname] = pg.PlotCurveItem()
                contain = False

            self.plot[dname].setData(**data)
            self.plot[dname].show()

            if not contain:
                self.get_view().addItem(self.plot[dname])

        elif self.plotter.dim == 3:
            if not self.plot.__contains__(dname):

                self.plot[dname] = gl.GLMeshItem()
                self.plot[dname].setGLOptions(option)
                contain = False

            self.plot[dname].setMeshData(**data)
            self.plot[dname].show()

            if not contain:
                self.get_view().addItem(self.plot[dname])

    def updateScatter(self, dname, data, option='opaque'):
        contain = True
        if self.plotter.dim == 2:
            if not self.plot.__contains__(dname):

                self.plot[dname] = pg.ScatterPlotItem()
                contain = False

        elif self.plotter.dim == 3:
            if not self.plot.__contains__(dname):

                self.plot[dname] = gl.GLScatterPlotItem()
                self.plot[dname].setGLOptions(option)
                contain = False

        self.plot[dname].setData(**data)
        self.plot[dname].show()

        if not contain:
            self.get_view().addItem(self.plot[dname])

    def updateLine(self, dname, data, option='opaque'):
        contain = True
        if self.plotter.dim == 2:
            if not self.plot.__contains__(dname):

                self.plot[dname] = pg.PlotCurveItem()
                contain = False

        elif self.plotter.dim == 3:
            if not self.plot.__contains__(dname):

                self.plot[dname] = gl.GLLinePlotItem()
                self.plot[dname].setGLOptions(option)
                contain = False

        self.plot[dname].setData(**data)
        self.plot[dname].show()

        if not contain:
            self.get_view().addItem(self.plot[dname])

    def hide_all(self):
        for p in self.plot.values():
            p.hide()

        self.t1_active = False
        self.t2_active = False

    def show_map(self):
        self.plot['obstacles'].show()

    def render_map(self):
        self.hide_all()
        self.updateMD(dname='obstacles', data=self.builder.map.plot(), option='opaque')
        self.show_map()

    def change_map(self):
        self.builder.new_map()

    def reset(self):
        self.builder.map.reset()

    def get_view(self):
        return self.plotter.view

    def get_widget(self):
        return self.plotter.get_widget()

    def get_map(self):
        return self.builder.get_map()

    def plot_tree(self, data):
        if data is self.t1Data:
            self.updateScatter('t1_points', data['points'], option='additive')
            self.updateLine('t1_lines', data['lines'], option='opaque')

            self.plot_leaves_t1()
            if self.t1_leaf_exists():
                self.hide('t1_leaves')

            self.plot_branches_t1()
            if self.t1_branch_exists():
                self.hide('t1_branches')

        else:
            self.updateScatter('t2_points', data['points'], option='additive')
            self.updateLine('t2_lines', data['lines'], option='opaque')

            self.plot_leaves_t2()
            if self.t2_leaf_exists():
                self.hide('t2_leaves')

            self.plot_branches_t2()
            if self.t2_branch_exists():
                self.hide('t2_branches')

    def t1_exists(self):
        return self.t1_active

    def t2_exists(self):
        return self.t2_active

    def t1_leaf_exists(self):
        return self.t1Data['leaves']

    def t1_branch_exists(self):
        return self.t1Data['branches']

    def t2_leaf_exists(self):
        return self.t2Data['leaves']

    def t2_branch_exists(self):
        return self.t2Data['branches']

    def plot_leaves_t1(self):
        self.updateMD('t1_leaves', self.t1Data['leaves'], option='additive')

    def plot_branches_t1(self):
        if self.t1Data['branches']:
            self.updateMD('t1_branches', self.t1Data['branches'], option='additive')

    def plot_leaves_t2(self):
        self.updateMD('t2_leaves', self.t2Data['leaves'], option='additive')

    def plot_branches_t2(self):
        if self.t2Data['branches']:
            self.updateMD('t2_branches', self.t2Data['branches'], option='additive')

    def hide(self, name):
        self.plot[name].hide()

    def show(self, name):
        self.plot[name].show()

    def set_tree_visibility(self, **kwargs):
        if kwargs['t1_leaves']:
            if self.t1_active and self.t1_leaf_exists():
                self.plot['t1_leaves'].show()

        if kwargs['t1_branches']:
            if self.t1_active and self.t1_branch_exists():
                self.plot['t1_branches'].show()

        if kwargs['t2_leaves']:
            if self.t2_active and self.t2_leaf_exists():
                self.plot['t2_leaves'].show()

        if kwargs['t2_branches']:
            if self.t2_active and self.t2_branch_exists():
                self.plot['t2_branches'].show()

    def plot_solution(self, solver):
        self.hide_all()
        self.show_map()

        self.t1Data = solver.t_start.plot()
        self.plot_tree(self.t1Data)
        self.t1_active = True

        if solver.t_end:
            self.t2Data = solver.t_end.plot()
            self.plot_tree(self.t2Data)
            self.t2_active = True

        else:
            self.t2_active = False

        map = self.get_map()
        path, points = map.plot_path(solver.path)

        if solver.path:
            self.updateLine('path', path, option='additive')
            self.updateScatter('path_points', points, option='additive')
            self.plot['path'].show()
            self.plot['path_points'].show()

















import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

from rrt_methods.RRTSolver import RRTSolver
from utils.maps.MapBuilder import MapBuilder
from utils.qt.plotting import PlotObject


class RRTApp():
    def __init__(self):
        self.solver = RRTSolver()
        self.plot = PlotObject()
        self.dim = 3

        self.opts = [
            dict(name='dim', type='list', limits=[2, 3], value=self.dim),
            dict(name='plan', type='list', limits=["rrt",
                                                   "rrt_connect",
                                                   "rrt_star",
                                                   "rrt_star_connect",
                                                   "multi_rrt_star_connect",
                                                   "informed_rrt_star",
                                                   "quick_rrt_star",
                                                   "informed_quick_rrt_star"],
                 value=self.solver.method),
            dict(name='step_size', type='float', value=20),
            dict(name='max_iter', type='float', value=1000),
            dict(name='map', type='list', limits=["rom"], value="rom", children=[
                dict(name='update', type='action')
            ]),
            dict(name='show', type='list', children=[
                dict(name='leaves', type='bool', value=self.solver.show_leaves),
                dict(name='branches', type='bool', value=self.solver.show_branches),
            ]),
            dict(name='run', type='action')
        ]

        self.params = pg.parametertree.Parameter.create(name='Parameters', type='group', children=self.opts)

        self.make_builder()
        self.params.child('dim').sigTreeStateChanged.connect(self.change_dim)
        self.params.child('step_size').sigTreeStateChanged.connect(self.set_step_size)
        self.params.child('max_iter').sigTreeStateChanged.connect(self.set_max_iter)
        self.params.child('map').sigTreeStateChanged.connect(self.make_builder)
        self.params.child('map').child('update').sigTreeStateChanged.connect(self.change_map)
        self.params.child('show').child('leaves').sigTreeStateChanged.connect(self.solver.leaf_toggle)
        self.params.child('show').child('branches').sigTreeStateChanged.connect(self.solver.branch_toggle)
        self.params.child('run').sigTreeStateChanged.connect(self.update)

        self.set_step_size()
        self.set_max_iter()
        self.plot.set_dim(self.params.child('dim').value())

        self.pt = pg.parametertree.ParameterTree()
        self.pt.setParameters(self.params)
        self.pt.setFixedSize(300, 900)

        self.splitter = QtWidgets.QSplitter()
        self.splitter.addWidget(self.pt)
        self.splitter.addWidget(self.plot.view)
        self.splitter.setFixedSize(1200, 900)
        self.splitter.show()

        pg.exec()

    def change_dim(self):
        self.plot.view.setParent(None)
        self.dim = self.params.child('dim').value()
        self.plot.set_dim(self.dim)
        self.splitter.addWidget(self.plot.view)

        self.make_builder()

    def make_builder(self):
        self.builder = MapBuilder(dim=self.params.child('dim').value(), seed=None, map=self.params.child('map').value())
        self.change_map()

    def set_step_size(self):
        self.solver.set_step_size(self.params.child('step_size').value())

    def set_max_iter(self):
        self.solver.set_max_iter(self.params.child('max_iter').value())

    def change_map(self):
        self.builder.new_map()
        self.solver.set_map(self.builder.map)

    def update(self):
        self.plot.clear()
        self.solver.set_method(self.params.child("plan").value())
        self.builder.map.reset()

        self.solver.run()
        if self.dim == 2:
            self.solver.plot(self.plot.view.getViewBox())

        else:
            self.solver.plot(self.plot.view)








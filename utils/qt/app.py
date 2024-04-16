import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPalette, QColor

from rrt_methods.RRTSolver import RRTSolver
from utils.maps.MapBuilder import MapBuilder
from utils.qt.plotting import PlotObject


class RRTApp():
    def __init__(self):
        self.solver = RRTSolver()
        self.plot = PlotObject()
        self.dim = 3
        self.plot.set_dim(self.dim)
        self.set_plot()
        self.builder = MapBuilder(3, None, "rom", 10, 150)
        self.change_map()
        self.builder.map.reset()

        self.opts = [
            dict(name='dim', type='list', limits=[2, 3], value=self.dim),
            dict(name='plan', type='list', limits=["rrt",
                                                   "rrt_connect",
                                                   "rrt_star",
                                                   "rrt_star_connect",
                                                   "multi_rrt_star_connect",
                                                   "informed_rrt_star",
                                                   "quick_rrt_star",
                                                   "informed_quick_rrt_star",
                                                   "ep_rrt_star",
                                                   "quick_ep_rrt_star"],
                 value=self.solver.method),
            dict(name='step_size', type='float', value=math.floor(self.dim * (self.builder.map.overlay.volume ** (1 / self.dim)) / 80)),
            dict(name='max_iter', type='float', value=1000),
            dict(name='map', type='list', limits=["rom"], value="rom", children=[
                dict(name='num_obstacles', type='float', value=10),
                dict(name='obstacle_size', type='float', value=150),
                dict(name='update', type='action'),
            ]),
            dict(name='show', type='list', children=[
                dict(name='leaves', type='bool', value=self.solver.show_leaves),
                dict(name='branches', type='bool', value=self.solver.show_branches),
            ]),
            dict(name='run', type='action'),
        ]

        self.params = pg.parametertree.Parameter.create(name='Parameters', type='group', children=self.opts)

        self.params.child('dim').sigTreeStateChanged.connect(self.change_dim)
        self.params.child('step_size').sigTreeStateChanged.connect(self.set_step_size)
        self.params.child('max_iter').sigTreeStateChanged.connect(self.set_max_iter)
        self.params.child('map').sigTreeStateChanged.connect(self.make_builder)
        self.params.child('map').child('update').sigTreeStateChanged.connect(self.change_map)
        self.params.child('map').child('num_obstacles').sigTreeStateChanged.connect(self.make_builder)
        self.params.child('map').child('obstacle_size').sigTreeStateChanged.connect(self.make_builder)
        self.params.child('show').child('leaves').sigTreeStateChanged.connect(self.solver.leaf_toggle)
        self.params.child('show').child('branches').sigTreeStateChanged.connect(self.solver.branch_toggle)
        self.params.child('run').sigTreeStateChanged.connect(self.update)

        self.set_step_size()
        self.set_max_iter()

        self.pt = pg.parametertree.ParameterTree()
        self.pt.setParameters(self.params)

        self.console = QtWidgets.QWidget()
        self.console.layout = QtWidgets.QVBoxLayout()
        self.console.setLayout(self.console.layout)

        self.time_display = QtWidgets.QLabel()
        self.time_display.setAlignment(Qt.AlignmentFlag.AlignJustify)

        self.d_time_display = QtWidgets.QLabel()
        self.d_time_display.setAlignment(Qt.AlignmentFlag.AlignRight)

        self.length_display = QtWidgets.QLabel()
        self.length_display.setAlignment(Qt.AlignmentFlag.AlignJustify)

        self.d_length_display = QtWidgets.QLabel()
        self.d_length_display.setAlignment(Qt.AlignmentFlag.AlignRight)

        self.data = QtWidgets.QWidget()
        self.data.layout = QtWidgets.QHBoxLayout()
        self.data.setLayout(self.data.layout)

        self.data_left = QtWidgets.QWidget()
        self.data_left_layout = QtWidgets.QVBoxLayout()
        self.data_left.setLayout(self.data_left_layout)

        self.data_right = QtWidgets.QWidget()
        self.data_right_layout = QtWidgets.QVBoxLayout()
        self.data_right.setLayout(self.data_right_layout)

        self.data_left_layout.addWidget(self.time_display)
        self.data_left_layout.addWidget(self.length_display)

        self.data_right_layout.addWidget(self.d_time_display)
        self.data_right_layout.addWidget(self.d_length_display)

        self.data.layout.addWidget(self.data_left)
        self.data.layout.addWidget(self.data_right)

        self.console.layout.addWidget(self.pt)
        self.console.layout.addWidget(self.data)

        self.splitter = QtWidgets.QSplitter()
        self.splitter.addWidget(self.console)
        self.splitter.addWidget(self.widget)
        self.splitter.setFixedSize(1200, 900)
        self.splitter.show()

        pg.exec()

    def update_display(self, prev_time, prev_length):
        s_time = self.solver.get_time()
        s_length = self.solver.get_length()
        time = self.solver.time
        length = self.solver.path_length
        palatte = QPalette()
        color = QColor()
        colors = palatte.ColorRole(1)
        palatte.setColor(colors.Text, color.red())

        if time > prev_time:
            self.d_time_display.setText("▲ " + f"{int(10000 * (time - prev_time)) / 10000}")
            self.d_time_display.setStyleSheet("QLabel { color : #ff0000; }")

        else:
            self.d_time_display.setText("▼ " + f"{int(10000 * (prev_time - time)) / 10000}")
            self.d_time_display.setStyleSheet("QLabel { color : #00ff00; }")

        if length > prev_length:
            self.d_length_display.setText("▲ " + f"{int(length - prev_length)}")
            self.d_length_display.setStyleSheet("QLabel { color : #ff0000; }")

        else:
            self.d_length_display.setText("▼ " + f"{int(prev_length - length)}")
            self.d_length_display.setStyleSheet("QLabel { color : #00ff00; }")

        self.time_display.setText(s_time)
        self.length_display.setText(s_length)

    def set_plot(self):
        self.view = self.plot.get_view()
        self.widget = self.plot.get_widget()

    def change_dim(self):
        self.widget.setParent(None)
        self.dim = self.params.child('dim').value()
        self.plot.set_dim(self.dim)
        self.set_plot()
        self.splitter.addWidget(self.widget)

        self.make_builder()

    def make_builder(self):
        self.builder = MapBuilder(self.params.child('dim').value(),
                                  None,
                                  self.params.child('map').value(),
                                  self.params.child('map').child('num_obstacles').value(),
                                  self.params.child('map').child('obstacle_size').value(),
                                  )
        self.change_map()

    def set_step_size(self):
        self.solver.set_step_size(self.params.child('step_size').value())

    def set_max_iter(self):
        self.solver.set_max_iter(self.params.child('max_iter').value())

    def change_map(self):
        self.plot.clear()
        self.builder.new_map()
        self.solver.set_map(self.builder.map)
        self.builder.map.plot(self.view)

    def update(self):
        prev_time = self.solver.time
        prev_length = self.solver.path_length
        self.plot.clear()
        self.solver.set_method(self.params.child("plan").value())
        self.builder.map.reset()

        self.solver.run()
        self.builder.map.plot(self.view)
        self.solver.plot(self.view)
        self.update_display(prev_time, prev_length)









import math
from rrt_methods.RRTSolver import RRTSolver
from utils.maps.MapBuilder import MapBuilder
from utils.qt.plotting import PlotObject
from PyQt6.QtGui import QColor, QPalette


class RRTCore():
    def __init__(self, console, display):
        self.solver = RRTSolver()
        self.builder = MapBuilder(3, None, "rom", 10, 150)
        self.plotter = PlotObject()
        self.console = console
        self.data = console.data
        self.display = display

        self.params = self.console.options.params
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
        self.params.child('step_size').setValue(math.floor(self.params.child('dim').value() * (self.builder.map.overlay.volume ** (1 / self.params.child('dim').value())) / 80))

        self.plotter.set_dim(3)
        self.update_view()
        self.change_map()
        self.update_display()
        self.builder.map.reset()

        self.set_step_size()
        self.set_max_iter()

    def update_display(self):
        self.display.connect(self.plotter.get_widget())

    def update_view(self):
        self.view = self.plotter.get_view()

    def change_dim(self):
        self.plotter.set_dim(self.params.child('dim').value())
        self.update_view()
        self.update_display()
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
        self.plotter.clear()
        self.builder.new_map()
        self.solver.set_map(self.builder.map)
        self.builder.map.plot(self.view)

    def update(self):
        prev_time = self.solver.time
        prev_length = self.solver.path_length
        self.plotter.clear()
        self.solver.set_method(self.params.child("plan").value())
        self.builder.map.reset()

        self.solver.run()
        self.builder.map.plot(self.view)
        self.solver.plot(self.view)
        self.update_data(prev_time, prev_length)

    def update_data(self, prev_time, prev_length):
        s_time = self.solver.get_time()
        s_length = self.solver.get_length()
        time = self.solver.time
        length = self.solver.path_length
        palatte = QPalette()
        color = QColor()
        colors = palatte.ColorRole(1)
        palatte.setColor(colors.Text, color.red())

        if time > prev_time:
            self.data.d_time_display.setText("▲ " + f"{int(10000 * (time - prev_time)) / 10000}")
            self.data.d_time_display.setStyleSheet("QLabel { color : #ff0000; }")

        else:
            self.data.d_time_display.setText("▼ " + f"{int(10000 * (prev_time - time)) / 10000}")
            self.data.d_time_display.setStyleSheet("QLabel { color : #00ff00; }")

        if length > prev_length:
            self.data.d_length_display.setText("▲ " + f"{int(length - prev_length)}")
            self.data.d_length_display.setStyleSheet("QLabel { color : #ff0000; }")

        else:
            self.data.d_length_display.setText("▼ " + f"{int(prev_length - length)}")
            self.data.d_length_display.setStyleSheet("QLabel { color : #00ff00; }")

        self.data.time_display.setText(s_time)
        self.data.length_display.setText(s_length)





from rrt_methods.RRTSolver import RRTSolver
from .main_window.display.plot_handler import PlotHandler
from .main_window.display.camera import Camera


class RRTCore():
    def __init__(self, console, display):
        self.solver = RRTSolver()
        self.plot_handler = PlotHandler()
        self.camera = Camera()
        self.console = console
        self.display = display
        self.data = console.data

        self.connect_pt()

        self.camera.connect(self.console.slider, self.console.buttons)
        self.update_map()
        self.camera.slider.sliderMoved.connect(self.focus_path)
        self.camera.slider.valueChanged.connect(self.focus_path)
        self.update_display()

        self.set_step_size()
        self.set_max_iter()

    def connect_pt(self):
        self.params = self.console.options.params
        self.params.child('dim').sigTreeStateChanged.connect(self.change_dim)
        self.params.child('step_size').sigTreeStateChanged.connect(self.set_step_size)
        self.params.child('max_iter').sigTreeStateChanged.connect(self.set_max_iter)
        self.params.child('map').child('update').sigTreeStateChanged.connect(self.update_map)
        self.params.child('map').child('num_obstacles').sigTreeStateChanged.connect(self.update_map)
        self.params.child('map').child('obstacle_size').sigTreeStateChanged.connect(self.update_map)
        self.params.child('run').sigTreeStateChanged.connect(self.update)
        self.params.child('step_size').setValue(40)

    def update_display(self):
        self.display.connect(self.plot_handler.get_widget())

    def focus_path(self):
        self.camera.follow_path()

    def change_dim(self):
        dim = self.params.child('dim').value()
        self.update_map()
        self.update_display()

        # hide the slider used to move along the 3d path
        if dim == 2:
            self.camera.hide_slider()

        else:
            self.camera.show_slider()

    def update_map(self):
        dim = self.params.child('dim').value()
        self.plot_handler.reconfig_builder(dim,
                                           None,
                                           self.params.child('map').value(),
                                           self.params.child('map').child('num_obstacles').value(),
                                           self.params.child('map').child('obstacle_size').value(),
                                           )
        self.plot_handler.switch_dim(self.params.child('dim').value())
        self.plot_handler.change_map()
        self.plot_handler.draw()
        self.solver.set_map(self.plot_handler.get_map())

        if dim == 3:
            self.camera.set_view(self.plot_handler.get_view())
            self.camera.reset()
            self.camera.disconnect()

    def set_step_size(self):
        self.solver.set_step_size(self.params.child('step_size').value())

    def set_max_iter(self):
        self.solver.set_max_iter(self.params.child('max_iter').value())

    def update(self):

        self.plot_handler.reset()

        prev_time = self.solver.get_time()
        prev_length = self.solver.get_length()
        self.solver.set_method(self.params.child('plan').value())
        self.solver.run(self.params.child('seed').value())

        self.plot_handler.draw()
        self.plot_handler.plot_solution(self.solver,
                                        self.params.child('show').child('branches').value(),
                                        self.params.child('show').child('leaves').value())
        self.console.update_data(self.solver.get_time(),
                                 self.solver.get_length(),
                                 prev_time,
                                 prev_length)

        self.camera.reset()
        self.camera.track_path(self.solver.path)










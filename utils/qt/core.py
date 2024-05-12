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

        self.variables = {}
        self.connect_pt()

        self.camera.connect(self.console.slider, self.console.buttons.buttons)
        self.new_map()
        self.update_display()

    def connect_pt(self):

        self.params = self.console.options.params

        self.params.child('plan').sigTreeStateChanged.connect(self.update_plan)
        self.params.child('plan').child('step_size').sigTreeStateChanged.connect(self.update_step_size)
        self.params.child('plan').child('max_iter').sigTreeStateChanged.connect(self.update_max_iter)
        self.params.child('plan').child('seed').sigTreeStateChanged.connect(self.update_seed)

        self.params.child('map').child('update').sigTreeStateChanged.connect(self.new_map)
        self.params.child('map').child('dim').sigTreeStateChanged.connect(self.swap_display)
        self.params.child('map').child('seed').sigTreeStateChanged.connect(self.update_seed)

        self.params.child('run').sigTreeStateChanged.connect(self.update)

        self.update_plan()
        self.update_step_size()
        self.update_max_iter()
        self.update_dim()
        self.update_seed()

    def update_display(self):
        self.display.connect(self.plot_handler.get_widget())

    # Update functions to update self.variables
    def update_plan(self):
        self.variables['method'] = self.params.child('plan').value()

    def update_step_size(self):
        self.variables['step_size'] = self.params.child('plan').child('step_size').value()

    def update_max_iter(self):
        self.variables['max_iter'] = self.params.child('plan').child('max_iter').value()

    def update_dim(self):
        self.variables['dim'] = self.params.child('map').child('dim').value()

    def update_map(self):
        self.variables['map'] = self.plot_handler.get_map()

    def update_seed(self):
        self.variables['map_seed'] = self.params.child('map').child('seed').value()
        self.variables['planner_seed'] = self.params.child('plan').child('seed').value()

    def swap_display(self):
        self.update_dim()
        self.new_map()
        self.update_display()

        # hide the slider used to move along the 3d path
        if self.variables['dim'] == 2:
            self.camera.hide()

        else:
            self.camera.show()

    def new_map(self):
        self.update_dim()
        self.plot_handler.reconfig_builder(self.variables['dim'],
                                           self.variables['map_seed'],
                                           self.params.child('map').value(),
                                           self.params.child('map').child('num_obstacles').value(),
                                           self.params.child('map').child('obstacle_size').value(),
                                           )
        self.plot_handler.switch_dim(self.variables['dim'])
        self.plot_handler.change_map()
        self.plot_handler.draw()
        self.update_map()

        if self.variables['dim'] == 3:
            self.camera.set_view(self.plot_handler.get_view())
            self.camera.reset()
            self.camera.disconnect()

    def update(self):

        self.plot_handler.reset()

        prev_time = self.solver.get_time()
        prev_length = self.solver.get_length()

        self.solver.set_data(self.variables)
        self.solver.run()

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























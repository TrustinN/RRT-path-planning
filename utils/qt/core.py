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

        self.params.child('trees').child('t_start').child('leaves').sigTreeStateChanged.connect(self.toggle_leaves_t1)
        self.params.child('trees').child('t_start').child('branches').sigTreeStateChanged.connect(self.toggle_branches_t1)

        self.params.child('trees').child('t_end').child('leaves').sigTreeStateChanged.connect(self.toggle_leaves_t2)
        self.params.child('trees').child('t_end').child('branches').sigTreeStateChanged.connect(self.toggle_branches_t2)

        self.params.child('trees').child('leaves').sigTreeStateChanged.connect(self.toggle_all_leaves)
        self.params.child('trees').child('branches').sigTreeStateChanged.connect(self.toggle_all_branches)

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

    def toggle_all_leaves(self):
        state = self.params.child('trees').child('leaves').value()

        self.params.child('trees').child('t_start').child('leaves').setValue(state)
        self.params.child('trees').child('t_end').child('leaves').setValue(state)

    def toggle_all_branches(self):
        state = self.params.child('trees').child('branches').value()

        self.params.child('trees').child('t_start').child('branches').setValue(state)
        self.params.child('trees').child('t_end').child('branches').setValue(state)

    def toggle_leaves_t1(self):
        if self.plot_handler.t1_exists():
            if self.params.child('trees').child('t_start').child('leaves').value():
                if self.plot_handler.t1_leaf_exists():
                    self.plot_handler.show('t1_leaves')
                    if self.params.child('trees').child('t_end').child('leaves').value():
                        self.params.child('trees').child('leaves').setValue(True)

            else:
                if self.plot_handler.t1_leaf_exists():
                    self.plot_handler.hide('t1_leaves')
                    restore = self.params.child('trees').child('t_end').child('leaves').value()
                    self.params.child('trees').child('leaves').setValue(False)
                    self.params.child('trees').child('t_end').child('leaves').setValue(restore)

    def toggle_branches_t1(self):
        if self.plot_handler.t1_exists():
            if self.params.child('trees').child('t_start').child('branches').value():
                if self.plot_handler.t1_branch_exists():
                    self.plot_handler.show('t1_branches')
                    if self.params.child('trees').child('t_end').child('branches').value():
                        self.params.child('trees').child('branches').setValue(True)

            else:
                if self.plot_handler.t1_branch_exists():
                    self.plot_handler.hide('t1_branches')
                    restore = self.params.child('trees').child('t_end').child('branches').value()
                    self.params.child('trees').child('branches').setValue(False)
                    self.params.child('trees').child('t_end').child('branches').setValue(restore)

    def toggle_leaves_t2(self):
        if self.plot_handler.t2_exists():
            if self.params.child('trees').child('t_end').child('leaves').value():
                if self.plot_handler.t2_leaf_exists():
                    self.plot_handler.show('t2_leaves')
                    if self.params.child('trees').child('t_start').child('leaves').value():
                        self.params.child('trees').child('leaves').setValue(True)

            else:
                if self.plot_handler.t2_leaf_exists():
                    self.plot_handler.hide('t2_leaves')
                    restore = self.params.child('trees').child('t_start').child('leaves').value()
                    self.params.child('trees').child('leaves').setValue(False)
                    self.params.child('trees').child('t_start').child('leaves').setValue(restore)

    def toggle_branches_t2(self):
        if self.plot_handler.t2_exists():
            if self.params.child('trees').child('t_end').child('branches').value():
                if self.plot_handler.t2_branch_exists():
                    self.plot_handler.show('t2_branches')
                    if self.params.child('trees').child('t_start').child('branches').value():
                        self.params.child('trees').child('branches').setValue(True)

            else:
                if self.plot_handler.t2_branch_exists():
                    self.plot_handler.hide('t2_branches')
                    restore = self.params.child('trees').child('t_start').child('branches').value()
                    self.params.child('trees').child('branches').setValue(False)
                    self.params.child('trees').child('t_start').child('branches').setValue(restore)

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
        self.plot_handler.render_map()
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

        self.plot_handler.plot_solution(self.solver)
        self.plot_handler.set_tree_visibility(**{
            't1_leaves': self.params.child('trees').child('t_start').child('leaves').value(),
            't1_branches': self.params.child('trees').child('t_start').child('branches').value(),
            't2_leaves': self.params.child('trees').child('t_end').child('leaves').value(),
            't2_branches': self.params.child('trees').child('t_end').child('branches').value(),
        })

        self.console.update_data(self.solver.get_time(),
                                 self.solver.get_length(),
                                 prev_time,
                                 prev_length)

        self.camera.reset()
        self.camera.track_path(self.solver.path)























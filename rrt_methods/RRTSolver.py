import timeit
import numpy as np
from .rrt import rrt_run as rrt
from .rrt_connect import rrt_run as rrt_connect
from .rrt_star import rrt_run as rrt_star
from .rrt_star_connect import rrt_run as rrt_star_connect
from .multi_rrt_star_connect import rrt_run as multi_rrt_star_connect
from .informed_rrt_star import rrt_run as informed_rrt_star
from .quick_rrt_star import rrt_run as quick_rrt_star
from .informed_quick_rrt_star import rrt_run as informed_quick_rrt_star
from .ep_rrt_star import rrt_run as ep_rrt_star
from .quick_ep_rrt_star import rrt_run as quick_ep_rrt_star
from .bto_rrt import rrt_run as bto_rrt
from .m_rrt import rrt_run as m_rrt


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
            "informed_quick_rrt_star": informed_quick_rrt_star,
            "ep_rrt_star": ep_rrt_star,
            "quick_ep_rrt_star": quick_ep_rrt_star,
            "bto_rrt": bto_rrt,
            "m_rrt": m_rrt,
        }
        self.time = 0
        self.path_length = 0

    def set_data(self, data):
        self.map = data['map']
        self.max_iter = data['max_iter']
        self.step_size = data['step_size']
        self.method = data['method']
        self.seed = data['planner_seed']

    def run(self):
        if self.seed:
            np.random.seed(self.seed)

        self.rrt_run = self.methods[self.method]

        time_start = timeit.default_timer()
        self.path, self.t_start, self.t_end = self.rrt_run(map=self.map, step_size=self.step_size, max_iter=self.max_iter)
        time_stop = timeit.default_timer()

        self.time = time_stop - time_start
        self.path_length = sum([np.linalg.norm(self.path[i] - self.path[i + 1]) for i in range(len(self.path) - 1)])

    def get_time(self):
        return self.time

    def get_length(self):
        return self.path_length





























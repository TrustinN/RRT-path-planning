import pyqtgraph as pg


class RRTOptions():
    def __init__(self):

        self.opts = [
            dict(name='dim', type='list', limits=[2, 3], value=3),
            dict(name='plan', type='list', limits=["rrt",
                                                   "rrt_connect",
                                                   "rrt_star",
                                                   "rrt_star_connect",
                                                   "multi_rrt_star_connect",
                                                   "informed_rrt_star",
                                                   "quick_rrt_star",
                                                   "informed_quick_rrt_star",
                                                   "ep_rrt_star",
                                                   "quick_ep_rrt_star",
                                                   "bto_rrt",
                                                   "m_rrt",
                                                   ],
                 value="rrt"),
            dict(name='step_size', type='float', value=50),
            dict(name='max_iter', type='int', value=1000),
            dict(name='map', type='list', limits=["rom", "maze"], value="rom", children=[
                dict(name='num_obstacles', type='int', value=10),
                dict(name='obstacle_size', type='float', value=150),
                dict(name='update', type='action'),
            ]),
            dict(name='show', type='list', children=[
                dict(name='leaves', type='bool', value=False),
                dict(name='branches', type='bool', value=False),
            ]),
            dict(name='seed', type='int', value=None),
            dict(name='run', type='action'),
        ]

        self.params = pg.parametertree.Parameter.create(name='Parameters', type='group', children=self.opts)

        self.pt = pg.parametertree.ParameterTree()
        self.pt.setParameters(self.params)


















import numpy as np
from utils.qt.app import RRTApp
from utils.quickhull.hull import QuickHull

# app = RRTApp()
# app.exec()



vertices = [np.array([291.66666667, 0., 0.]),
            np.array([308.33333333, 0., 0.]),
            np.array([291.66666667, 100., 0.]),
            np.array([291.66666667, 0., 100.]),
            np.array([308.33333333, 100., 0.]),
            np.array([308.33333333, 0., 100.]),
            np.array([291.66666667, 100., 100.]),
            np.array([308.33333333, 100., 100.])]

QuickHull(vertices)



























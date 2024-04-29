import numpy as np
from rrt_methods.rrt_utils import parametric_spline, spline_eval
import pyqtgraph as pg
from PyQt6.QtGui import QVector3D
from test_utils import plot_axis, make_plot, plot_path
 

pg.mkQApp("test")
view = make_plot(100, pos=QVector3D(700, 700, 400), distance=500)
plot_axis(100, view)


path = np.array([np.array([10, 10, 10]),
                 np.array([12.70930894, 13.76004323, 10.72270329]),
                 np.array([14.49136346, 15.43512949, 11.31830654]),
                 np.array([21.22126982, 21.76101138, 13.56760454]),
                 np.array([41.22906404, 40.56411409, 20.25527423]),
                 np.array([58.73512796, 57.01541947, 26.10684692]),
                 np.array([66.2195247, 64.04880486, 28.6085908]),
                 np.array([531.64262813, 501.89084802, 184.9045472]),
                 np.array([640.94414205, 605.71829274, 223.16799303]),
                 np.array([710.53329129, 690.12310137, 272.35269509]),
                 np.array([755.03859445, 746.249382, 311.63788113]),
                 np.array([759.91261879, 750.89233051, 318.20329305]),
                 np.array([812.11320808, 797.68102653, 390.60213053]),
                 np.array([815.84640338, 795.79212686, 399.49357128]),
                 np.array([822.72947969, 791.86129499, 416.17484714]),
                 np.array([820.30090527, 791.32281097, 440.25295027]),
                 np.array([793.20686862, 789.47665509, 745.89036666]),
                 np.array([789.76968536, 789.64877548, 788.27800908]),
                 np.array([789.76147993, 789.64918638, 788.37919916]),
                 np.array([790, 790, 790])])

plot_path(path, view)
pn = parametric_spline(np.array(path))
plot_path(pn, view)

x = path.T[0]
y = path.T[1]
z = path.T[2]

pn = spline_eval(np.linspace(10, 790, len(x)), x)
plot_path(pn, view)
#
pn = spline_eval(np.linspace(10, 790, len(y)), y)
plot_path(pn, view)
# pn = parametric_spline(np.array([np.array([p[0], p[1]]) for p in path]))
# plot_path(pn, view)

pn = spline_eval(np.linspace(10, 790, len(z)), z)
pn = [np.array([0, v[0], v[1]]) for v in pn]
plot_path(pn, view)


pg.exec()













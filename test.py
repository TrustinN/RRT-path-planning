import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6.QtGui import QVector3D
from test_utils import plot_axis, make_plot
from colorutils import Color
 

pg.mkQApp("test")
view = make_plot(10, pos=QVector3D(0, 0, 0), distance=10)
plot_axis(10, view)

points = np.array([np.array([1, 1, 1]),
                   np.array([1, 2, 3]),
                   np.array([3, 6, 2]),
                   ])
rgb = Color(web="#ff0000").rgb
i = np.array([np.array(rgb) / 255])
k = np.repeat(i, 3, axis=0)
print(k)
points = gl.GLScatterPlotItem(pos=points,
                              color=k)
view.addItem(points)

print(np.zeros(3))
print(np.concatenate((np.array([np.zeros(3)]), k)))



pg.exec()













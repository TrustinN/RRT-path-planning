import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from utils.quickhull.hull import QuickHull

app = pg.mkQApp("Oblique")
view = gl.GLViewWidget()
view.setCameraPosition(distance=1000)
view.show()

g = gl.GLGridItem()
view.addItem(g)


x = np.array([1, 0, 0])
y = np.array([0, 1, 0])
z = np.array([0, 0, 1])


v = [np.array([291.66666667,   0.,   0.]), np.array([308.33333333,   0.,   0.]),
     np.array([291.66666667, 100.,   0.]), np.array([291.66666667,   0., 100.]),
     np.array([308.33333333, 100.,   0.]), np.array([308.33333333,   0., 100.]),
     np.array([291.66666667, 100., 100.]), np.array([308.33333333, 100., 100.])]

hull = QuickHull(v)
hull.plot(view)




pg.exec()






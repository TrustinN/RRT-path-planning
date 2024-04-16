import timeit
import numpy as np
from map_utils import SampleScope
import pyqtgraph as pg
import pyqtgraph.opengl as gl

app = pg.mkQApp("Oblique")
view = gl.GLViewWidget()
view.show()

g = gl.GLGridItem()
view.addItem(g)


x = np.array([1, 0, 0])
y = np.array([0, 1, 0])
z = np.array([0, 0, 1])


# p0 = np.array([1, -1, 1])
p0 = np.array([0, 0, 0])
p1 = np.array([3, 2, 5])
p_base0 = np.array([3, 5, 1])
p_base0 = p_base0

l_base0 = gl.GLLinePlotItem(pos=np.array([p0, 5 * p_base0 + p0]), color=pg.mkColor("#ff0000"))
view.addItem(l_base0)

l_base2 = gl.GLLinePlotItem(pos=np.array([p0, 5 * p1 + p0]), color=pg.mkColor("#0000ff"))
view.addItem(l_base2)


oblique = SampleScope.ObliqueCylinder(p0, p1, p_base0 - p0)

start = timeit.default_timer()
points = []
for i in range(10000):
    points.append(oblique.sample())

stop = timeit.default_timer()
print(stop - start)

scatter = gl.GLScatterPlotItem(pos=np.array(points))
view.addItem(scatter)

pg.exec()




import numpy as np
from rrt_methods.bto_rrt import parametric_spline
import pyqtgraph.opengl as gl
import pyqtgraph as pg
 

pg.mkQApp("test")
view = gl.GLViewWidget()
view.setCameraParams(distance=2)
view.pan(2, 0, 0)
view.show()

g = gl.GLGridItem()
view.addItem(g)

x = np.array([10, 0, 0])
y = np.array([0, 10, 0])
origin = np.array([0, 0, 0])

line = gl.GLLinePlotItem(pos=np.array([x, origin]), color=pg.mkColor("#ff0000"))
view.addItem(line)

line = gl.GLLinePlotItem(pos=np.array([y, origin]), color=pg.mkColor("#0000ff"))
view.addItem(line)

# x = np.linspace(0, 10, num=50)
# y = np.exp(-x / 2) * np.sin(2 * x)
x = np.array([0, 1, 1, 2, 2, 1])
y = np.array([0, 0, 1, 1, 0, -1])

path = np.array([x, y]).T
plot_data = [path[0]] + [path[i // 2] for i in range(len(path * 2))] + [path[-1]]
line = gl.GLLinePlotItem(pos=np.array(plot_data), mode='lines')
view.addItem(line)

path = parametric_spline(x, y, np.ones(len(x)))
plot_data = [path[0]] + [path[i // 2] for i in range(len(path * 2))] + [path[-1]]
line = gl.GLLinePlotItem(pos=np.array(plot_data), mode='lines')
view.addItem(line)







pg.exec()









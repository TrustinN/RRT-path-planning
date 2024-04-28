import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6.QtGui import QVector3D


def make_plot(scale, pos=QVector3D(0, 0, 0), distance=100):

    view = gl.GLViewWidget()
    view.setCameraPosition(pos=pos, distance=distance)
    view.show()

    g = gl.GLGridItem()
    g.scale(scale, scale, scale)
    view.addItem(g)

    return view


def plot_axis(scale, view, origin=np.array([0, 0, 0])):
    x = np.array([scale, 0, 0])
    y = np.array([0, scale, 0])
    z = np.array([0, 0, scale])

    line = gl.GLLinePlotItem(pos=np.array([x + origin, origin]), color=pg.mkColor("#ff0000"))
    view.addItem(line)

    line = gl.GLLinePlotItem(pos=np.array([y + origin, origin]), color=pg.mkColor("#00ff00"))
    view.addItem(line)

    line = gl.GLLinePlotItem(pos=np.array([z + origin, origin]), color=pg.mkColor("#0000ff"))
    view.addItem(line)


def plot_path(path, view):
    for i in range(len(path) - 1):
        line = gl.GLLinePlotItem(pos=np.array([path[i], path[i + 1]]),
                                 color=pg.mkColor("#ff00ff"),
                                 width=3,)
        line.setGLOptions("opaque")
        view.addItem(line)
    point = gl.GLScatterPlotItem(pos=np.array(path), size=10, color=pg.mkColor("#ff0000"))
    view.addItem(point)


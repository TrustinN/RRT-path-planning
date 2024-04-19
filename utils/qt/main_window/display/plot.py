import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6.QtGui import QVector3D


class PlotObject():
    def __init__(self):
        pg.mkQApp("Map")

        self.widget2D = pg.plot()
        self.widget3D = gl.GLViewWidget()
        self.view = self.widget2D
        self.dim = 2

    def reset_camera(self):
        if self.dim == 3:
            self.view.setCameraPosition(pos=QVector3D(10, 10, 10), distance=1500, azimuth=225, elevation=30)

    def set_dim(self, dim):
        self.dim = dim
        self.show_plot()

    def show_plot(self):
        if self.dim == 2:
            self.view.hide()
            self.view = self.widget2D
            self.view.show()

        elif self.dim == 3:
            self.view.hide()
            self.view = self.widget3D
            self.view.show()

        self.reset_camera()

    def get_widget(self):
        return self.view

    def get_view(self):
        if self.dim == 2:
            return self.view.getViewBox()

        elif self.dim == 3:
            return self.view

    def clear(self):
        if self.dim == 2:
            self.view.getViewBox().clear()

        elif self.dim == 3:
            self.view.clear()
            g = gl.GLGridItem()
            g.translate(400, 400, -200)
            g.scale(100, 100, 100)
            self.view.addItem(g)










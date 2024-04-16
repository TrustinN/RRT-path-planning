import pyqtgraph as pg
import pyqtgraph.opengl as gl


class PlotObject():
    def __init__(self):
        pg.mkQApp("Map")

        self.d2 = pg.plot()
        self.d2.setFixedSize(900, 900)

        self.d3 = gl.GLViewWidget()
        self.d3.setFixedSize(900, 900)
        self.d3.setCameraPosition(distance=1500)
        self.d3.pan(400, 400, 400)

        self.view = self.d2

    def set_dim(self, dim):
        self.dim = dim
        self.show_plot()

    def show_plot(self):
        if self.dim == 2:
            self.view.hide()
            self.view = self.d2
            self.view.show()

        elif self.dim == 3:
            self.view.hide()
            self.view = self.d3
            self.view.show()

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










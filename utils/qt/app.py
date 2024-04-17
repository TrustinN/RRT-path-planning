import pyqtgraph as pg

from .main_window.main_window import MainWindow
from .main_window.graph import RRTDisplay
from .main_window.console.console import RRTConsole
from .main_window.console.core import RRTCore


class RRTApp():
    def __init__(self):
        pg.mkQApp("RRT-Path-Planning")

        self.main_window = MainWindow()
        self.graph = RRTDisplay()
        self.console = RRTConsole()

        self.core = RRTCore(self.console, self.graph)

        self.main_window.window.addWidget(self.console.window)
        self.main_window.window.addWidget(self.graph.window)
        self.main_window.window.setFixedSize(1200, 900)
        self.main_window.window.show()

    def run(self):
        pg.exec()













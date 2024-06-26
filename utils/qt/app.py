from PyQt6.QtWidgets import QApplication

from .main_window.main_window import MainWindow
from .main_window.display.graph import RRTDisplay
from .main_window.console.console import RRTConsole
from .core import RRTCore


class RRTApp(QApplication):
    def __init__(self):
        super().__init__([])

        self.main_window = MainWindow()
        self.graph = RRTDisplay()
        self.console = RRTConsole()

        self.core = RRTCore(self.console, self.graph)

        self.main_window.attach(self.console)
        self.main_window.attach(self.graph)
        self.main_window.show()














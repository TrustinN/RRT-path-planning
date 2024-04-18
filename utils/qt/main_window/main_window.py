from PyQt6.QtWidgets import QMainWindow
from pyqtgraph.Qt import QtWidgets


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.window = QtWidgets.QSplitter()
        self.setWindowTitle("RRT-Path-Planning")
        self.setCentralWidget(self.window)






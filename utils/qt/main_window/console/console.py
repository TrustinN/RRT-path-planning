from pyqtgraph.Qt import QtWidgets

from .data import RRTData
from .parameter_tree import RRTOptions
from .path_traverser import PathTraverser


class RRTConsole():

    def __init__(self):
        self.window = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout()
        self.window.setLayout(self.layout)
        self.window.setFixedSize(300, 900)

        self.data = RRTData()
        self.options = RRTOptions()
        self.traverser = PathTraverser()

        self.layout.addWidget(self.options.pt)
        self.layout.addWidget(self.traverser)
        self.layout.addWidget(self.data.window)

















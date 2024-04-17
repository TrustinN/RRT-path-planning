from pyqtgraph.Qt import QtWidgets

from .data import RRTData
from .parameter_tree import RRTOptions


class RRTConsole():

    def __init__(self):
        self.window = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout()
        self.window.setLayout(self.layout)
        self.window.setFixedSize(300, 900)

        self.data = RRTData()
        self.options = RRTOptions()

        self.layout.addWidget(self.options.pt)
        self.layout.addWidget(self.data.window)

















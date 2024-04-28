from pyqtgraph.Qt import QtWidgets


class RRTDisplay(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QtWidgets.QHBoxLayout()
        self.setLayout(self.layout)
        self.widget = None

    def connect(self, widget):

        if self.widget:
            self.widget.setParent(None)
            self.layout.addWidget(widget)
            self.widget = widget

        else:
            self.layout.addWidget(widget)







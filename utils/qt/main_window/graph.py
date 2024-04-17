from pyqtgraph.Qt import QtWidgets


class RRTDisplay():
    def __init__(self):
        self.window = QtWidgets.QWidget()
        self.layout = QtWidgets.QHBoxLayout()
        self.window.setLayout(self.layout)
        self.widget = None

    def connect(self, widget):

        if self.widget:
            self.widget.setParent(None)
            self.layout.addWidget(widget)
            self.widget = widget

        else:
            self.layout.addWidget(widget)







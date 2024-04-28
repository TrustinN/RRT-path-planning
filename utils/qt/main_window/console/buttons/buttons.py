from pyqtgraph.Qt import QtWidgets
from .play_button import PlayButton
from .switch_button import SwitchButton


class Buttons(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QtWidgets.QHBoxLayout()
        self.setLayout(self.layout)
        self.buttons = [PlayButton(),
                        SwitchButton()]

        for b in self.buttons:
            self.layout.addWidget(b)


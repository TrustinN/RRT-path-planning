from pyqtgraph.Qt import QtWidgets

from .data import RRTData
from .parameter_tree import RRTOptions
from .path_traverser import PathTraverser
from .play_button import PlayButton


class RRTConsole():

    def __init__(self):
        self.window = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout()
        self.window.setLayout(self.layout)
        self.window.setFixedSize(300, 900)

        self.data = RRTData()
        self.options = RRTOptions()
        self.slider = PathTraverser()
        self.button = PlayButton()

        self.layout.addWidget(self.options.pt)
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.data.window)

    def update_data(self, new_time, new_length, prev_time, prev_length):

        if new_time > prev_time:
            self.data.d_time_display.setText("▲ " + f"{int(10000 * (new_time - prev_time)) / 10000}")
            self.data.d_time_display.setStyleSheet("QLabel { color : #ff0000; }")

        else:
            self.data.d_time_display.setText("▼ " + f"{int(10000 * (prev_time - new_time)) / 10000}")
            self.data.d_time_display.setStyleSheet("QLabel { color : #00ff00; }")

        if new_length > prev_length:
            self.data.d_length_display.setText("▲ " + f"{int(new_length - prev_length)}")
            self.data.d_length_display.setStyleSheet("QLabel { color : #ff0000; }")

        else:
            self.data.d_length_display.setText("▼ " + f"{int(prev_length - new_length)}")
            self.data.d_length_display.setStyleSheet("QLabel { color : #00ff00; }")

        self.data.time_display.setText(f"Time: {int(new_time * 10000) / 10000} seconds")
        self.data.length_display.setText(f"Path Length: {int(new_length)}")

















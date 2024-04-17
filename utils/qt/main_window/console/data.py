from pyqtgraph.Qt import QtWidgets
from PyQt6.QtCore import Qt


class RRTData():

    def __init__(self):
        self.time_display = QtWidgets.QLabel()
        self.time_display.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self.d_time_display = QtWidgets.QLabel()
        self.d_time_display.setAlignment(Qt.AlignmentFlag.AlignRight)

        self.length_display = QtWidgets.QLabel()
        self.length_display.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self.d_length_display = QtWidgets.QLabel()
        self.d_length_display.setAlignment(Qt.AlignmentFlag.AlignRight)

        self.window = QtWidgets.QWidget()
        self.layout = QtWidgets.QHBoxLayout()
        self.window.setLayout(self.layout)

        self.data_left = QtWidgets.QWidget()
        self.data_left_layout = QtWidgets.QVBoxLayout()
        self.data_left.setLayout(self.data_left_layout)

        self.data_right = QtWidgets.QWidget()
        self.data_right_layout = QtWidgets.QVBoxLayout()
        self.data_right.setLayout(self.data_right_layout)

        self.data_left_layout.addWidget(self.time_display)
        self.data_left_layout.addWidget(self.length_display)

        self.data_right_layout.addWidget(self.d_time_display)
        self.data_right_layout.addWidget(self.d_length_display)

        self.layout.addWidget(self.data_left)
        self.layout.addWidget(self.data_right)















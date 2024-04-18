from PyQt6.QtWidgets import QSlider
from PyQt6.QtCore import Qt


class PathTraverser(QSlider):
    def __init__(self):
        super().__init__()

        self.setMinimum(0)
        self.setMaximum(100)
        self.setOrientation(Qt.Orientation.Horizontal)

    def slider_position(self, p):
        return p







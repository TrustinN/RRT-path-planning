from PyQt6.QtWidgets import QSlider
from PyQt6.QtCore import Qt
from PyQt6.QtCore import QPropertyAnimation


class PathTracker(QSlider):
    def __init__(self):
        super().__init__()

        self.min = 0
        self.max = 300
        self.reset()
        self.setMinimum(self.min)
        self.setMaximum(self.max)
        self.setOrientation(Qt.Orientation.Horizontal)
        self.anim = QPropertyAnimation(self, b"value")

    def reset(self):
        self.start = self.min
        self.end = self.max
        self.reversed = False

    def reverse(self):
        self.start = self.max - self.start
        self.end = self.max - self.end
        self.reversed = not self.reversed

    def slider_position(self, p):
        return p

    def animate(self):
        self.anim.setDuration(4000)
        self.anim.setStartValue(self.start)
        self.anim.setEndValue(self.end)
        self.anim.start()


















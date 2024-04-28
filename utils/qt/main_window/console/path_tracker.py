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
        self.setSliderPosition(self.min)

    def reverse(self):
        self.start = self.max - self.start
        self.end = self.max - self.end
        self.reversed = not self.reversed
        self.setSliderPosition(self.sliderPosition())

    def can_animate(self):
        return self.sliderPosition() != self.end

    def animate(self):
        if self.can_animate():
            percentage = self.sliderPosition() / (self.max - self.min)
            time = int(4000 * (1 - percentage))
            if self.reversed:
                time = int(4000 * percentage)
            self.anim.setDuration(time)
            self.anim.setStartValue(self.sliderPosition())
            self.anim.setEndValue(self.end)
            self.anim.start()

















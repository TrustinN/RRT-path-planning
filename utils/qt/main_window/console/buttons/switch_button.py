from PyQt6.QtWidgets import QPushButton
from PyQt6.QtGui import QIcon
import utils.qt.Resources.resources


class SwitchButton(QPushButton):
    def __init__(self):
        super().__init__()
        self.setIcon(QIcon(":/icons/switch.png"))
        self.setFixedWidth(50)





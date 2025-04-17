from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal
from typing import Optional


class DeviceBar(QWidget):
    connected = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._connected = False

    def device_connected(self, state: bool):
        self._connected = state
        self.connected.emit(state)

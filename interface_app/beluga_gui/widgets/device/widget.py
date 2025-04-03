from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import QObject, pyqtSignal
from typing import Optional


class DeviceBar(QWidget):
    class DeviceConnectedEmitter(QObject):
        bool_update = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._connected = False
        self._connect = self.DeviceConnectedEmitter()
        self.connected = {"bool": self._connect.bool_update }

    def device_connected(self, state: bool):
        self._connected = state
        self._connect.bool_update.emit(state)
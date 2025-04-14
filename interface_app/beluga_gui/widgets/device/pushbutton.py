from PyQt5.QtWidgets import QPushButton, QWidget
from PyQt5.QtCore import pyqtSignal
from typing import Optional


class DeviceConnectButton(QPushButton):
    connected = pyqtSignal([bool], [str])

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._current_port = ""

    def update_port(self, port: str):
        self._current_port = port
        if self._current_port:
            self.setEnabled(True)
        else:
            self.setEnabled(False)

    def update_connected(self, connected: bool, status: str):
        if connected:
            self.setText("Disconnect")
            self.connected[str].emit(status)
        else:
            self.setText("Connect")
            self.connected[str].emit(status)

        self.connected[bool].emit(connected)

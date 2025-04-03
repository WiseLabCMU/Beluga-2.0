from PyQt5.QtWidgets import QPushButton, QWidget
from PyQt5.QtCore import QObject, pyqtSignal
from typing import Optional


class DeviceConnectButton(QPushButton):
    class DeviceConnectedEmitter(QObject):
        bool_update = pyqtSignal(bool)
        str_update = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._current_port = ""
        self._connected = self.DeviceConnectedEmitter()
        self.connected = {
            "bool": self._connected.bool_update,
            "QString": self._connected.str_update
        }
        self.state = False # TODO: Get rid of this

    def update_port(self, port: str):
        self._current_port = port
        print(port)
        if self._current_port:
            self.setEnabled(True)
        else:
            self.setEnabled(False)

    def update_connected(self):
        # TODO: take in bool and str inputs and handle pressed signal from main application
        self.state = not self.state

        if self.state:
            self.setText("Disconnect")
            self._connected.str_update.emit("") # TODO: Set this to an error
        else:
            self.setText("Connect")
            self._connected.str_update.emit("Status") # TODO: Set this to an error

        self._connected.bool_update.emit(self.state)
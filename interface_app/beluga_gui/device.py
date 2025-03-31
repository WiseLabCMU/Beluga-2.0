from PyQt5.QtWidgets import QLabel, QComboBox, QPushButton, QWidget
from typing import Optional, Iterable
from PyQt5 import QtCore


class DeviceComboBox(QComboBox):
    class UpdateBuddyEmitter(QtCore.QObject):
        update_bool = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._buddy_update = self.UpdateBuddyEmitter()
        self.update_buddy = {"bool": self._buddy_update.update_bool}

    def update_device_list(self, options: Iterable[str]):
        current = self.currentText()
        items = [""] + list(options)
        index = None
        if current in items:
            index = items.index(current)
        self.clear()
        self.addItems(items)
        if index is not None:
            self.setCurrentIndex(index)

    def lock(self, connect_state: bool):
        enable = not connect_state
        self.setEnabled(enable)
        self._buddy_update.update_bool.emit(enable)

class DeviceConnectButton(QPushButton):
    class DeviceConnectedEmitter(QtCore.QObject):
        bool_update = QtCore.pyqtSignal(bool)
        str_update = QtCore.pyqtSignal(str)

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

class DeviceBar(QWidget):
    class DeviceConnectedEmitter(QtCore.QObject):
        bool_update = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._connected = False
        self._connect = self.DeviceConnectedEmitter()

    def device_connected(self, state: bool):
        self._connected = state
        self._connect.bool_update.emit(state)

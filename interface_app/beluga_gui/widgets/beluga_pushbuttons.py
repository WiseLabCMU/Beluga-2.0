from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QPushButton, QWidget
from .beluga_widget import BelugaWidgetBase
from typing import Optional, Callable


class BelugaPushButton(QPushButton):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._handler: Optional[Callable[[], None]] = None
        self.pressed.connect(self.button_pressed)

    def set_pressed_handler(self, handler: Optional[Callable[[], None]] = None):
        self._handler = handler

    def button_pressed(self):
        if self._handler is not None:
            self._handler()


class BleButton(BelugaPushButton, BelugaWidgetBase):
    class BleUpdateEmitter(QObject):
        bool_update = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._ble_update = self.BleUpdateEmitter()
        self.ble_update = {"bool": self._ble_update.bool_update}

    def set_ble_state(self, running: bool):
        self._ble_running = running
        if self._ble_running:
            self.setText("Stop BLE")
        else:
            self.setText("Start BLE")
        self._ble_update.bool_update.emit(self._ble_running)

    def update(self):
        enable = not self._uwb_running
        self.setEnabled(enable)

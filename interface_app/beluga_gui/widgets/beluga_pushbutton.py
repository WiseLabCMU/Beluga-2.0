from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QPushButton, QWidget
from .beluga_widget_base import BelugaWidgetBase
from typing import Optional, Callable


DEBUG = True


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
        if DEBUG:
            self._handler = self.test_handler

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

    def test_handler(self):
        state = not self._ble_running
        self.set_ble_state(state)


class UwbButton(BelugaPushButton, BelugaWidgetBase):
    class UwbUpdateEmitter(QObject):
        bool_update = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._uwb_update = self.UwbUpdateEmitter()
        self.uwb_update = {"bool": self._uwb_update.bool_update}
        if DEBUG:
            self._handler = self.test_handler

    def set_uwb_state(self, running: bool):
        self._uwb_running = running
        if self._uwb_running:
            self.setText("Stop UWB")
        else:
            self.setText("Start UWB")
        self._uwb_update.bool_update.emit(self._uwb_running)

    def update(self):
        enable = self._ble_running
        self.setEnabled(enable)

    def test_handler(self):
        state = not self._uwb_running
        self.set_uwb_state(state)


class RebootButton(BelugaPushButton):
    pass

class ApplyPowerButton(BelugaPushButton):
    def update_checkbox_state(self, state: bool):
        self.setEnabled(not state)

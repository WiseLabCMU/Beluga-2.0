from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QPushButton, QWidget
from .beluga_widget_base import BelugaWidgetBase
from typing import Optional, Callable


DEBUG = False


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
    ble_update = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        if DEBUG:
            self._handler = self.test_handler

    def set_ble_state(self, running: bool):
        self._ble_running = running
        if self._ble_running:
            self.setText("Stop BLE")
        else:
            self.setText("Start BLE")
        self.ble_update.emit(self._ble_running)

    def update(self):
        enable = not self._uwb_running
        self.setEnabled(enable)

    def test_handler(self):
        state = not self._ble_running
        self.set_ble_state(state)


class UwbButton(BelugaPushButton, BelugaWidgetBase):
    uwb_update = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        if DEBUG:
            self._handler = self.test_handler

    def set_uwb_state(self, running: bool):
        self._uwb_running = running
        if self._uwb_running:
            self.setText("Stop UWB")
        else:
            self.setText("Start UWB")
        self.uwb_update.emit(self._uwb_running)

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


class StartRangingButton(BelugaPushButton, BelugaWidgetBase):
    _ranging = False

    def update(self):
        self._ranging = self.uwb_running and self.ble_running
        if self._ranging:
            self.setText("Stop Ranging")
        else:
            self.setText("Start Ranging")
        self.setEnabled(self._connected)

    @property
    def ranging(self):
        return self._ranging

    def update_connected_state(self, state):
        self._connected = state
        self.update()


class CaptureDataBtn(BelugaPushButton, BelugaWidgetBase):
    def update(self):
        self.setEnabled(self._connected)

    def update_connected_state(self, state):
        self._connected = state
        self.update()

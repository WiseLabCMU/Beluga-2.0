from PyQt5.QtCore import pyqtSignal


class BelugaWidgetBase:
    buddy_update = pyqtSignal(bool)
    _connected = True
    _ble_running = False
    _uwb_running = False

    def __init__(self):
        self._uwb_running = False
        self._ble_running = False

    def update_ble_state(self, ble_state: bool):
        self._ble_running = ble_state
        self.update()

    def update_uwb_state(self, uwb_state: bool):
        self._uwb_running = uwb_state
        self.update()

    def update(self):
        pass

    @property
    def ble_running(self) -> bool:
        return self._ble_running

    @property
    def uwb_running(self) -> bool:
        return self._uwb_running

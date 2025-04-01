from PyQt5.QtCore import QObject, pyqtSignal


class BelugaWidgetBase:
    class BuddyStateUpdateEmitter(QObject):
        bool_update = pyqtSignal(bool)
    _connected = True
    _ble_running = True
    _uwb_running = False

    def __init__(self):
        self._uwb_running = False
        self._ble_running = False
        self._buddy_sig = self.BuddyStateUpdateEmitter()
        self.buddy_update = {"bool": self._buddy_sig.bool_update }

    def update_ble_state(self, ble_state: bool):
        self._ble_running = ble_state
        self.update()

    def update_uwb_state(self, uwb_state: bool):
        self._uwb_running = uwb_state
        self.update()

    def update(self):
        pass

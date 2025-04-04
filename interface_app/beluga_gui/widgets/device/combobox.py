from PyQt5.QtWidgets import QWidget, QComboBox
from PyQt5.QtCore import QObject, pyqtSignal
from typing import Optional, Iterable


class DeviceComboBox(QComboBox):
    class UpdateBuddyEmitter(QObject):
        update_bool = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._buddy_update = self.UpdateBuddyEmitter()
        self.update_buddy = {"bool": self._buddy_update.update_bool}

    def update_device_list(self, options: Iterable[str]):
        if not self.isEnabled():
            return
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

from PyQt5.QtWidgets import QWidget, QComboBox
from PyQt5.QtCore import pyqtSignal
from typing import Optional, Iterable


class DeviceComboBox(QComboBox):
    update_buddy = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

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
        self.update_buddy.emit(enable)

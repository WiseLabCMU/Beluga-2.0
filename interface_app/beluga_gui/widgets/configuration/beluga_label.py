from PyQt5.QtWidgets import QLabel, QWidget
from .beluga_widget_base import BelugaWidgetBase
from typing import Optional


class BelugaLabel(QLabel, BelugaWidgetBase):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._buddy_state = False

    def update_buddy_state(self, state: bool):
        self._buddy_state = state
        self.update()

    def update(self):
        enable = self._buddy_state
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

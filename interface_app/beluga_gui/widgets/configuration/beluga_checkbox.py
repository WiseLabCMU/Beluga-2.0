from PyQt5.QtWidgets import QCheckBox, QWidget
from .beluga_widget_base import BelugaWidgetBase
from typing import Optional, Callable


class BelugaCheckBox(QCheckBox):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._handler: Optional[Callable[[bool], None]] = None
        self.toggled.connect(self.handle_toggle)

    def set_toggle_handler(self, handler: Optional[Callable[[bool], None]] = None):
        self._handler = handler

    def handle_toggle(self, state: bool):
        if self._handler is not None:
            self._handler(state)


class RangingCheckBox(BelugaCheckBox):
    pass


class LedCheckBox(BelugaCheckBox):
    pass


class UwbCheckBox(BelugaCheckBox, BelugaWidgetBase):
    def update(self):
        self.setEnabled(not self._uwb_running)
        self.buddy_update.emit(not self._uwb_running)


class UwbSfdCheckBox(UwbCheckBox):
    pass


class AntennaCheckBox(BelugaCheckBox):
    _hardware_support = False

    def supported(self, support):
        self._hardware_support = support

    def setEnabled(self, a0):
        super().setEnabled(a0 and self._hardware_support)


class UwbPhrCheckBox(UwbCheckBox):
    pass

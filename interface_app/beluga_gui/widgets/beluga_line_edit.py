from PyQt5.QtWidgets import QLineEdit, QWidget
from .beluga_widget_base import BelugaWidgetBase
from typing import Optional, Callable


class BelugaLineEdit(QLineEdit):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._handler: Optional[Callable[[str], None]] = None
        self.returnPressed.connect(self.handle_return_pressed)

    def set_handler(self, handler: Optional[Callable[[str], None]] = None):
        self._handler = handler

    def handle_return_pressed(self):
        print(self.text())
        if self._handler is not None:
            self._handler(self.text())


class UwbLineEdit(BelugaLineEdit, BelugaWidgetBase):
    def update(self):
        self.setEnabled(not self._uwb_running)
        self._buddy_sig.bool_update.emit(not self._uwb_running)


class NodeIdLineEdit(UwbLineEdit):
    pass


class PollRateLineEdit(BelugaLineEdit):
    pass


class TimeoutLineEdit(BelugaLineEdit):
    pass


class UwbPanIdLineEdit(UwbLineEdit):
    pass

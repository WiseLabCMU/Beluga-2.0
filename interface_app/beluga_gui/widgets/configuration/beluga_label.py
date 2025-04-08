from PyQt5.QtWidgets import QLabel, QWidget
from .beluga_widget_base import BelugaWidgetBase
from typing import Optional, Union


class BelugaLabel(QLabel, BelugaWidgetBase):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._buddy_state = True

    def update_buddy_state(self, state: bool):
        self._buddy_state = state
        self.update()

    def update(self):
        enable = self._buddy_state
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

    def setEnabled(self, a0):
        super().setEnabled(a0 and self._buddy_state)


class BelugaConfigLabel(QLabel):
    def update_config(self, new: Union[str, bool]):
        prefix = self.property("prefix")

        if isinstance(new, str):
            suffix = self.property("suffix")
            if suffix is None:
                suffix = ""
            config_prefix = self.property("config_prefix")
            if config_prefix is None:
                config_prefix = ""
            self.setText(f"{prefix}: {config_prefix}{new} {suffix}".strip())
        if isinstance(new, bool):
            if new:
                new = self.property("true_str")
            else:
                new = self.property("false_str")
            self.setText(f"{prefix}: {new}")

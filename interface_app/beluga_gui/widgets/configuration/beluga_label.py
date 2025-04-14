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


class BelugaStatusLabel(QLabel):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._connected = False
        self._port = ""
        self._version = ""
        self.setText("Disconnected")

    def set_connected(self, state):
        self._connected = state
        self.update()

    @property
    def port(self):
        return self._port

    @port.setter
    def port(self, new_port):
        self._port = new_port
        self.update()

    @property
    def version(self):
        return self._version

    @version.setter
    def version(self, new_version):
        self._version = new_version
        self.update()

    def update(self):
        if self._connected:
            status = f"Connected to {self._port}. Node version {self._version}"
        else:
            status = "Disconnected"
        self.setText(status)

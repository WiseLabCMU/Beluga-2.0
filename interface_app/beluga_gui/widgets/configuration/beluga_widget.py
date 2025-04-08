from PyQt5.QtWidgets import QWidget, QCheckBox
from .beluga_pushbutton import UwbButton, ApplyPowerButton
from typing import Optional


class UwbCustomTxPower(QWidget):
    def update_checkbox_state(self, state: bool):
        self.setEnabled(not state)
        for child in self.findChildren(QWidget):
            child.setEnabled(not state)


class SettingsWidget(QWidget):
    _connected = False
    _rebooting = False

    def update_connected_state(self, connected: bool):
        self._connected = connected
        self.update()

    @property
    def rebooting(self):
        return self._rebooting

    @rebooting.setter
    def rebooting(self, val: bool):
        self._rebooting = val
        self.update()

    def update(self):
        enable = self._connected and not self._rebooting
        self.setEnabled(enable)
        tx_power_checkbox: Optional[QCheckBox] = None
        for child in self.findChildren(QWidget):
            if not enable or not isinstance(child, UwbButton):
                child.setEnabled(enable)
            if child.objectName() == "uwb_tx_power_checkbox":
                tx_power_checkbox = child
        if tx_power_checkbox is not None:
            tx_power_checkbox.toggled.emit(tx_power_checkbox.isChecked())

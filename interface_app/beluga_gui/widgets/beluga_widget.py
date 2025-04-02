from PyQt5.QtWidgets import QWidget
from .beluga_pushbutton import UwbButton, ApplyPowerButton


class UwbCustomTxPower(QWidget):
    def update_checkbox_state(self, state: bool):
        self.setEnabled(not state)
        for child in self.findChildren(QWidget):
            child.setEnabled(not state)


class SettingsWidget(QWidget):
    def update_connected_state(self, connected: bool):
        self.setEnabled(connected)
        tx_power_checkbox = None
        for child in self.findChildren(QWidget):
            if not connected or not isinstance(child, UwbButton):
                child.setEnabled(connected)
            if child.objectName() == "uwb_tx_power_checkbox":
                tx_power_checkbox = child
        if tx_power_checkbox is not None:
            tx_power_checkbox.setChecked(True)
            tx_power_checkbox.toggled.emit(True)


    # TODO: Set states for UWB stuff

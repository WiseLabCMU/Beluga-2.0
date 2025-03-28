from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QCheckBox, QPushButton, QComboBox, QDoubleSpinBox, QLineEdit, QLabel
from PyQt5.QtWidgets import QWidget
from PyQt5 import QtCore
from typing import Optional


class AmplifierComboBox(QComboBox):
    pass

class AntennaCheckBox(QCheckBox):
    pass

class ApplyPowerButton(QCheckBox):
    pass

class BleButton(QPushButton):
    class BleButtonSignalEmitter(QObject):
        bool_update = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._connected = False
        self._uwb_running = False
        self._ble_running = False
        self._ble_emitter = self.BleButtonSignalEmitter()
        self.ble_updated = {"bool": self._ble_emitter.bool_update}

    def dev_connected(self):
        self._connected = not self._connected
        self.update()

    def toggle_ble(self):
        self._ble_running = not self._ble_running
        self.update()
        self._ble_emitter.bool_update.emit(self._ble_running)

    def update_uwb(self, uwb_state: bool):
        self._uwb_running = uwb_state

        self.update()

    def update(self):
        enable = self._connected and not self._uwb_running
        if self._ble_running:
            self.setText("Stop BLE")
        else:
            self.setText("Start BLE")
        self.setEnabled(enable)

class BootModeComboBox(QComboBox):
    pass

class ChannelComboBox(QComboBox):
    pass

class CoarseGainComboBox(QComboBox):
    pass

class FineGainSpinBox(QDoubleSpinBox):
    pass

class LedCheckBox(QCheckBox):
    pass

class NeighborEvictionSchemeComboBox(QComboBox):
    pass

class NodeIdLineEdit(QLineEdit):
    def __init__(self, parent):
        super().__init__(parent)
        self._device_connected = False
        self._uwb_active = False

    def dev_connected(self):
        self._device_connected = not self._device_connected
        self.update()

    def uwb_update(self, state: bool):
        self._uwb_active = state
        self.update()

    def update(self):
        enable = self._device_connected and not self._uwb_active
        self.setEnabled(enable)

class PollRateLineEdit(QLineEdit):
    pass

class RangingCheckBox(QCheckBox):
    pass

class RebootButton(QPushButton):
    pass

class TimeoutLineEdit(QLineEdit):
    pass

class UwbButton(QPushButton):
    class UwbButtonSignalEmitter(QObject):
        bool_update = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._dev_connected = False
        self._uwb_active = False
        self._ble_active = False
        self._uwb_updated = self.UwbButtonSignalEmitter()
        self.uwb_updated = {'bool': self._uwb_updated.bool_update}

    def dev_connected(self):
        self._dev_connected = not self._dev_connected
        self.update()

    def update_uwb(self):
        self._uwb_active = not self._uwb_active
        self.update()
        self._uwb_updated.bool_update.emit(self._uwb_active)

    def update_ble(self, state: bool):
        self._ble_active = state
        self.update()

    def update(self):
        enable = self._dev_connected and self._ble_active
        if self._uwb_active:
            self.setText("Stop UWB")
        else:
            self.setText("Start UWB")
        self.setEnabled(enable)



class UwbDataRateComboBox(QComboBox):
    pass

class UwbPacSizeComboBox(QComboBox):
    pass

class UwbPanIdLineEdit(QLineEdit):
    pass

class UwbPhrCheckBox(QCheckBox):
    pass

class UwbPowerConfigCheckBox(QCheckBox):
    pass

class UwbPowerLabel(QLabel):
    pass

class UwbPreambleLengthComboBox(QComboBox):
    pass

class UwbPulseRateComboBox(QComboBox):
    pass

class UwbTxPowerComboBox(QComboBox):
    pass

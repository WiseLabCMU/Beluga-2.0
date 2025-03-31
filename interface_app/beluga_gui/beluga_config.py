from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QCheckBox, QPushButton, QComboBox, QDoubleSpinBox, QLineEdit, QLabel
from PyQt5.QtWidgets import QWidget
from PyQt5 import QtCore
from typing import Optional


class AbstractBelugaWidget:
    _connected = False
    def dev_connected(self):
        self._connected = not self._connected
        self.update()


class AmplifierComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class AntennaCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class ApplyPowerButton(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class BleButton(QPushButton, AbstractBelugaWidget):
    class BleButtonSignalEmitter(QObject):
        bool_update = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._uwb_running = False
        self._ble_running = False
        self._ble_emitter = self.BleButtonSignalEmitter()
        self.ble_updated = {"bool": self._ble_emitter.bool_update}

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

class BootModeComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class ChannelComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class CoarseGainComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class FineGainSpinBox(QDoubleSpinBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class LedCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class NeighborEvictionSchemeComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class NodeIdLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self._uwb_active = False

    def uwb_update(self, state: bool):
        self._uwb_active = state
        self.update()

    def update(self):
        enable = self._connected and not self._uwb_active
        self.setEnabled(enable)

class PollRateLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class RangingCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class RebootButton(QPushButton, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class TimeoutLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbButton(QPushButton, AbstractBelugaWidget):
    class UwbButtonSignalEmitter(QObject):
        bool_update = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._uwb_active = False
        self._ble_active = False
        self._uwb_updated = self.UwbButtonSignalEmitter()
        self.uwb_updated = {'bool': self._uwb_updated.bool_update}

    def update_uwb(self):
        self._uwb_active = not self._uwb_active
        self.update()
        self._uwb_updated.bool_update.emit(self._uwb_active)

    def update_ble(self, state: bool):
        self._ble_active = state
        self.update()

    def update(self):
        enable = self._connected and self._ble_active
        if self._uwb_active:
            self.setText("Stop UWB")
        else:
            self.setText("Start UWB")
        self.setEnabled(enable)



class UwbDataRateComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbPacSizeComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbPanIdLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbPhrCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbPowerConfigCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbPowerLabel(QLabel, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbPreambleLengthComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbPulseRateComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class UwbTxPowerComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class BelugaCustomPowerLabel(QLabel, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

class BelugaLabel(QLabel, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)

from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QCheckBox, QPushButton, QComboBox, QDoubleSpinBox, QLineEdit, QLabel
from PyQt5.QtWidgets import QWidget
from PyQt5 import QtCore
from typing import Optional


class AbstractBelugaWidget:
    class BuddyStateUpdateEmitter(QObject):
        bool_update = QtCore.pyqtSignal(bool)
    _connected = True
    _ble_running = True
    _uwb_running = False

    def __init__(self):
        self._uwb_running = False
        self._ble_running = False
        self._connected = False
        self._buddy_sig = self.BuddyStateUpdateEmitter()
        self.buddy_update = {"bool": self._buddy_sig.bool_update }

    def dev_connected(self):
        self._connected = not self._connected
        self.update()

    def update_ble_state(self, ble_state: bool):
        self._ble_running = ble_state
        self.update()

    def update_uwb_state(self, uwb_state: bool):
        self._uwb_running = uwb_state
        self.update()

    def update(self):
        pass


class AmplifierComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class AntennaCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class ApplyPowerButton(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class BleButton(QPushButton, AbstractBelugaWidget):
    class BleButtonSignalEmitter(QObject):
        bool_update = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._ble_emitter = self.BleButtonSignalEmitter()
        self.ble_updated = {"bool": self._ble_emitter.bool_update}

    def toggle_ble(self):
        self._ble_running = not self._ble_running
        self.update()
        self._ble_emitter.bool_update.emit(self._ble_running)

    def update(self):
        enable = self._connected and not self._uwb_running
        if self._ble_running:
            self.setText("Stop BLE")
        else:
            self.setText("Start BLE")
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class BootModeComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class ChannelComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class CoarseGainComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class FineGainSpinBox(QDoubleSpinBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class LedCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class NeighborEvictionSchemeComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class NodeIdLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent):
        super().__init__(parent)

    def update(self):
        enable = self._connected and not self._uwb_running
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class PollRateLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class RangingCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class RebootButton(QPushButton, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class TimeoutLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbButton(QPushButton, AbstractBelugaWidget):
    class UwbButtonSignalEmitter(QObject):
        bool_update = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._uwb_updated = self.UwbButtonSignalEmitter()
        self.uwb_updated = {'bool': self._uwb_updated.bool_update}

    def update_uwb(self):
        self._uwb_running = not self._uwb_running
        self.update()
        self._uwb_updated.bool_update.emit(self._uwb_running)

    def set_uwb_state(self, state: bool):
        self._uwb_running = state
        self.update()
        self._uwb_updated.bool_update.emit(self._uwb_running)

    def update(self):
        enable = self._connected and self._ble_running
        if self._uwb_running:
            self.setText("Stop UWB")
        else:
            self.setText("Start UWB")
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbDataRateComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected and not self._uwb_running
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbPacSizeComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected and not self._uwb_running
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbPanIdLineEdit(QLineEdit, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbPhrCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected and not self._uwb_running
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbPowerConfigCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbPowerLabel(QLabel, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbPreambleLengthComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected and not self._uwb_running
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbPulseRateComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected and not self._uwb_running
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class UwbTxPowerComboBox(QComboBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class BelugaCustomPowerLabel(QLabel, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enable = self._connected
        self.setEnabled(enable)
        self._buddy_sig.bool_update.emit(enable)

class BelugaLabel(QLabel, AbstractBelugaWidget):
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

class UwbSfdCheckBox(QCheckBox, AbstractBelugaWidget):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)

    def update(self):
        enabled = self._connected and not self._uwb_running
        self.setEnabled(enabled)
        self._buddy_sig.bool_update.emit(enable)

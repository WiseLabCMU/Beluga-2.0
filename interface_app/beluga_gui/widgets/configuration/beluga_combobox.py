from .beluga_widget_base import BelugaWidgetBase
from PyQt5.QtWidgets import QComboBox, QWidget
from typing import Optional, Callable


class BelugaComboBoxBase(QComboBox):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._handler: Optional[Callable[[int], None]] = None
        self.currentIndexChanged.connect(self.handle_index_changed)

    def set_changed_index_handler(self, handler: Optional[Callable[[None], int]] = None):
        self._handler = handler

    def handle_index_changed(self, new_index):
        if self._handler is not None:
            self._handler(new_index)


class AmplifierComboBox(BelugaComboBoxBase, BelugaWidgetBase):
    _hardware_support = False

    def supported(self, support: bool):
        self._hardware_support = support
        self._buddy_sig.bool_update.emit(self._hardware_support)

    def setEnabled(self, a0):
        super().setEnabled(a0 and self._hardware_support)


class BootModeComboBox(BelugaComboBoxBase):
    pass


class UwbComboBoxBase(BelugaComboBoxBase, BelugaWidgetBase):
    def update(self):
        self.setEnabled(not self._uwb_running)
        self._buddy_sig.bool_update.emit(not self._uwb_running)


class ChannelComboBox(UwbComboBoxBase):
    pass


class UwbDataRateComboBox(UwbComboBoxBase):
    pass


class UwbPulseRateComboBox(UwbComboBoxBase):
    pass


class UwbPreambleLengthComboBox(UwbComboBoxBase):
    pass


class UwbPacSizeComboBox(UwbComboBoxBase):
    pass


class UwbTxPowerComboBox(BelugaComboBoxBase, BelugaWidgetBase):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._simple_power = True

    def update_checkbox_state(self, state: bool):
        self._simple_power = state
        self.setEnabled(self._simple_power)
        self._buddy_sig.bool_update.emit(self._simple_power)
        if self._simple_power:
            self.refresh_advanced_power()

    def refresh_advanced_power(self):
        self.currentIndexChanged.emit(self.currentIndex())


class NeighborEvictionSchemeComboBox(BelugaComboBoxBase, BelugaWidgetBase):
    _support = False

    def supported(self, support: bool):
        self._support = support
        self._buddy_sig.bool_update.emit(self._support)

    @property
    def support(self):
        return self._support

    def setEnabled(self, a0):
        super().setEnabled(a0 and self._support)


class CoarseGainComboBox(QComboBox):
    def update_power(self, power: int):
        stage = self.property("power_stage")
        mask = 0x7
        shift = (8 * stage) + 5
        setting = (power >> shift) & mask
        index = (~setting) & mask
        self.setCurrentIndex(index)

    def simple_power_update(self, mode: int):
        if mode == 0:
            power = 0x0E080222
        else:
            power = 0x1F1F1F1F
        self.update_power(power)

    @property
    def stage(self):
        return self.property("power_stage")

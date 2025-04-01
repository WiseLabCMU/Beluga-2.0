from .beluga_widget import BelugaWidgetBase
from PyQt5.QtWidgets import QComboBox, QWidget
from typing import Optional, Callable


class BelugaComboBoxBase(QComboBox):
    def __init__(self, parent: Optional[QWidget]):
        super().__init__(parent)
        self._handler: Optional[Callable[[None], int]] = None
        self.currentIndexChanged.connect(self.handle_index_changed)

    def set_current_mode(self, index: int):
        self.setCurrentIndex(index)

    def set_changed_index_handler(self, handler: Optional[Callable[[None], int]] = None):
        self._handler = handler

    def handle_index_changed(self, new_index):
        if self._handler is not None:
            self._handler(new_index)


class AmplifierComboBox(BelugaComboBoxBase):
    pass


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

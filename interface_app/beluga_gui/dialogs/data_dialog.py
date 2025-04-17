from .base.data_dialog import Ui_Dialog
from .messages import ErrorMessage
from.file_dialog import FileDialog, FileFilter
from PyQt5.QtWidgets import QDialog, QWidget
from typing import Optional


class DataGatheringDialog(QDialog):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.ui.file_select.pressed.connect(self.launch_file_dialog)
        self._file: Optional[str] = None

    def launch_file_dialog(self):
        dialog = FileDialog(self, [FileFilter("meas", "Measurement files"), FileFilter("log", "Log files")])
        file_path = dialog.exec()
        self.ui.trial_name.setText(file_path)
        self._file = file_path

    def accept(self):
        if not self.ui.trial_name.text():
            ErrorMessage(self, "File input", "Please enter file name")
            self.ui.trial_name.setFocus()
            return
        super().accept()

    @property
    def save_file(self):
        return self._file

    @property
    def samples(self):
        return self.ui.samples.value()

    @property
    def timeout(self):
        return self.ui.timeout.value()

from .base.data_dialog import Ui_Dialog
from .messages import ErrorMessage
from PyQt5.QtWidgets import QDialog, QWidget, QMessageBox
from typing import Optional


class DataGatheringDialog(QDialog):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)

    def accept(self):
        if not self.ui.trial_name.text():
            ErrorMessage(self, "File input", "Please enter file name")
            self.ui.trial_name.setFocus()
            return
        super().accept()

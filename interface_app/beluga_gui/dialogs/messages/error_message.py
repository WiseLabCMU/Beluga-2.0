from PyQt5.QtWidgets import QMessageBox, QWidget
from typing import Optional


class ErrorMessage(QMessageBox):
    def __init__(self, caption: str, message: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle(caption)
        self.setText(message)
        self.setIcon(QMessageBox.Critical)
        self.show()
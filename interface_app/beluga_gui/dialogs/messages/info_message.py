from PyQt5.QtWidgets import QMessageBox, QWidget
from typing import Optional


class InfoMessage(QMessageBox):
    def __init__(self, parent: Optional[QWidget] = None, caption: str = "Info", message: str = ""):
        super().__init__(parent)
        if not message:
            raise ValueError("Invalid error message")
        self.setWindowTitle(caption)
        self.setText(message)
        self.setIcon(QMessageBox.Information)
        self.show()
from PyQt5.QtWidgets import QWidget, QPlainTextEdit
from typing import Optional


class BelugaTerminal(QPlainTextEdit):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)

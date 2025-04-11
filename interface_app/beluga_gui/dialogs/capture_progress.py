from PyQt5.QtWidgets import QProgressDialog, QWidget
from PyQt5.QtCore import QTimer, Qt
from .messages import ErrorMessage
from typing import Optional


class CaptureProgress(QProgressDialog):
    def __init__(self, parent: Optional[QWidget] = None, samples: int = 1000, timeout: int = 5):
        super().__init__(parent)
        self._samples = samples
        self.setWindowModality(Qt.WindowModal)
        self.setWindowTitle("Ranging")
        self.setLabelText("Capturing ranges...")
        self._progress = 0
        self.setMinimumDuration(0)
        self.setMaximum(samples)
        self.setAutoClose(True)
        self._timeout = QTimer()
        self._timeout.singleShot(timeout * 1000 * 60, self.timeout)

    def timeout(self):
        self.cancel()
        ErrorMessage(self, "Timeout", "Data capture timed out")

from PyQt5.QtWidgets import QLabel, QComboBox, QPushButton, QWidget
from typing import Optional


class ConnectStatusMsg(QLabel):
    pass


class DeviceComboBox(QComboBox):
    def update(self):
        self.setEnabled(not self.isEnabled())

class DeviceConnectButton(QPushButton):
    pass

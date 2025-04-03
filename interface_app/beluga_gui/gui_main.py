import sys
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QThread, QObject
from typing import Optional, Callable, Iterable
from beluga_gui import Ui_BelugaGUI
from beluga_serial import BelugaSerial, BelugaSerialAttr
from copy import deepcopy
import time
from serial import Serial, SerialException


class BelugaGui:
    class PortUpdateCheck(QThread):
        def __init__(self, serial: BelugaSerial, update: Callable[[Iterable[str]], None], parent: Optional[QObject] = None):
            super().__init__(parent)
            self._serial = serial
            self._callback = update

        def run(self):
            prev_iter = None
            while True:
                targets = self._serial.find_ports()

                ports: Iterable[str] = []
                for target in targets:
                    ports += [f"{target}: {port}" for port in targets[target]]
                ports.sort()
                eligible = []

                for port in ports:
                    port_: str = port.rsplit(":", 1)[-1]
                    try:
                        s = Serial(port_.lstrip(), baudrate=115200, exclusive=True)
                        s.close()
                    except SerialException:
                        pass
                    else:
                        eligible.append(port)

                if eligible != prev_iter:
                    prev_iter = deepcopy(eligible)
                    self._callback(prev_iter)
                time.sleep(1.0)


    def __init__(self, argv):
        self.app = QApplication(argv)
        self.window = QMainWindow()
        self.ui = Ui_BelugaGUI()
        self.ui.setupUi(self.window)
        self.serial = BelugaSerial(BelugaSerialAttr(auto_connect=False))
        self._port_update = self.PortUpdateCheck(self.serial, self.ui.device_combobox.update_device_list)
        self._port_update.start()


    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())



if __name__ == "__main__":
    gui = BelugaGui(sys.argv)
    gui.run()

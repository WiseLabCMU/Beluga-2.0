import sys
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QThread, QObject
from typing import Optional, Callable, Iterable, Tuple
from beluga_gui import Ui_BelugaGUI
from beluga_serial import BelugaSerial, BelugaSerialAttr
from copy import deepcopy
import time
from serial import Serial, SerialException
import math


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
        self._connected = False
        self.ui.connect_button.pressed.connect(self.update_connection_state)


    def run(self):
        self.window.show()
        ret = self.app.exec_()
        self.serial.close()
        sys.exit(ret)

    @staticmethod
    def strtoint(response: str) -> int:
        return int("".join([c for c in response if c.isdigit()]))

    @staticmethod
    def extract_hex(response: str) -> int:
        index = response.find("0x")
        if index == -1:
            raise ValueError("Hex number not present")
        hex_str = response[index + 2:].split()[0]
        print(hex_str)

    @staticmethod
    def strip_alpha(response: str):
        return "".join([c for c in response if c.isdigit()])

    def channel_to_index(self, channel: str) -> int:
        channel = self.strtoint(channel) - 1
        if channel == 7:
            channel -= 1
        return channel

    def preamble_to_index(self, preamble):
        length = self.strtoint(preamble)
        if length < 2048:
            return int(math.ceil(math.log2(length))) - 6
        else:
            return int(math.log2(length)) - 5

    def _gather_beluga_data(self) -> Tuple[bool, str]:
        # See if we can get the ID
        try:
            id_ = self.serial.id()
        except Exception:
            return False, "Unable to communicate with port"
        if id_ == 'Response timed out':
            return False, "Unable to communicate with port"
        # TODO: Create something to get current comms state
        self.ui.node_id_line_edit.setText(self.strip_alpha(id_))
        mode = self.serial.bootmode()
        mode = self.strtoint(mode)
        self.ui.boot_mode_combobox.setCurrentIndex(mode)
        rate = self.serial.rate()
        self.ui.rate_line_edit.setText(self.strip_alpha(rate))
        channel = self.serial.channel()
        index = self.channel_to_index(channel)
        self.ui.channel_combobox.setCurrentIndex(index)
        timeout = self.serial.timeout()
        self.ui.timeout_line_edit.setText(self.strip_alpha(timeout))
        mode = self.serial.twrmode()
        self.ui.ranging_checkbox.setChecked(bool(self.strtoint(mode)))
        mode = self.serial.ledmode()
        self.ui.led_checkbox.setChecked(not bool(self.strtoint(mode)))
        mode = self.serial.phr()
        self.ui.phr_checkbox.setChecked(bool(self.strtoint(mode)))
        mode = self.serial.pwramp()
        try:
            mode = self.strtoint(mode)
            self.ui.extern_amp_combobox.setCurrentIndex(mode)
            self.ui.extern_amp_combobox.supported(True)
        except ValueError:
            self.ui.extern_amp_combobox.supported(False)
        mode = self.serial.antenna(1)
        try:
            # TODO:
            mode = self.channel_to_index(mode)  # Antenna is 1-indexed
            self.ui.antenna_checkbox.setChecked(bool(mode))
            self.ui.antenna_checkbox.supported(True)
        except ValueError:
            self.ui.antenna_checkbox.supported(False)
        rate = self.serial.datarate()
        self.ui.uwb_datarate_combobox.setCurrentIndex(self.strtoint(rate))
        rate = self.serial.pulserate()
        self.ui.uwb_pulserate_combobox.setCurrentIndex(self.strtoint(rate))
        mode = self.serial.phr()
        self.ui.phr_checkbox.setChecked(bool(self.strtoint(mode)))
        preamble = self.serial.preamble()
        self.ui.uwb_preamble_combobox.setCurrentIndex(self.preamble_to_index(preamble))
        pac = self.serial.pac()
        self.ui.uwb_pac_combobox.setCurrentIndex(self.strtoint(pac))
        pan = self.serial.panid()
        self.extract_hex(pan)
        # TODO: Set UI elements
        return True, ""

    def update_connection_state(self):
        if self._connected:
            try:
                self.serial.close()
            except Exception as e:
                # TODO: Bring up error window
                print(e)
                sys.exit(1)
            self.ui.connect_button.update_connected(False, "")
            self._connected = False
        else:
            port = self.ui.device_combobox.currentText()
            port_: str = port.rsplit(":", 1)[-1].lstrip()
            try:
                self.serial.open_target(port_)
            except FileNotFoundError:
                # TODO: Error window
                print(port_)
                print("Port not found")
            except ValueError:
                self.serial.close()
                self.ui.connect_button.update_connected(False, "Unable to communicate with port")
            else:
                status, error = self._gather_beluga_data()
                self.ui.connect_button.update_connected(status, error)
                self._connected = status




if __name__ == "__main__":
    gui = BelugaGui(sys.argv)
    gui.run()

import sys
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QThread, QObject, QThreadPool, QRunnable
from typing import Optional, Callable, Iterable, Tuple
from beluga_gui import Ui_BelugaGUI
from beluga_serial import BelugaSerial, BelugaSerialAttr, BelugaStatus, unpack_beluga_version
from copy import deepcopy
import time
from serial import Serial, SerialException
import math


class BelugaGui:
    class PortUpdateCheck(QThread):
        def __init__(self, serial: BelugaSerial, update: Callable[[Iterable[str]], None],
                     parent: Optional[QObject] = None):
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
        self.ui.ble_button.pressed.connect(self.update_ble)
        self.ui.uwb_button.pressed.connect(self.update_uwb)
        self.ui.node_id_line_edit.set_handler(self.update_node_id)
        self.ui.boot_mode_combobox.set_changed_index_handler(self.update_bootmode)
        self.ui.rate_line_edit.set_handler(self.update_rate)
        self.ui.channel_combobox.set_changed_index_handler(self.update_channel)
        self.ui.timeout_line_edit.set_handler(self.update_timeout)
        self.ui.ranging_checkbox.set_toggle_handler(self.update_ranging)
        self.ui.led_checkbox.set_toggle_handler(self.update_leds)
        self.ui.sfd_checkbox.set_toggle_handler(self.update_sfd)
        self.ui.extern_amp_combobox.set_changed_index_handler(self.update_amplifiers)
        self.ui.antenna_checkbox.set_toggle_handler(self.update_antenna)
        self.ui.uwb_datarate_combobox.set_changed_index_handler(self.update_datarate)
        self.ui.uwb_pulserate_combobox.set_changed_index_handler(self.update_pulserate)
        self.ui.phr_checkbox.set_toggle_handler(self.update_phr)
        self.ui.uwb_preamble_combobox.set_changed_index_handler(self.update_preamble)
        self.ui.uwb_pac_combobox.set_changed_index_handler(self.update_pac)
        self.ui.pan_id_text_edit.set_handler(self.update_pan)
        self.ui.eviction_combobox.set_changed_index_handler(self.update_evict)
        self.ui.reboot_button.pressed.connect(self.reboot)
        self.ui.uwb_txpower_combobox.set_changed_index_handler(self.update_simple_power)
        self.ui.apply_power.pressed.connect(self.update_complex_power)

    def run(self):
        self.window.show()
        ret = self.app.exec_()
        self.serial.close()
        sys.exit(ret)

    def refresh_ble(self):
        status = self.serial.status()
        status = BelugaStatus(status)
        self.ui.ble_button.set_ble_state(status.ble)

    def refresh_uwb(self):
        status = self.serial.status()
        status = BelugaStatus(status)
        self.ui.uwb_button.set_uwb_state(status.uwb)

    def refresh_id(self):
        id_ = self.serial.id()
        self.ui.node_id_line_edit.setText(self.strip_alpha(id_))

    def refresh_bootmode(self):
        mode = self.serial.bootmode()
        mode = self.strtoint(mode)
        self.ui.boot_mode_combobox.setCurrentIndex(mode)

    def refresh_rate(self):
        rate = self.serial.rate()
        self.ui.rate_line_edit.setText(self.strip_alpha(rate))

    def refresh_channel(self):
        channel = self.serial.channel()
        index = self.channel_to_index(channel)
        self.ui.channel_combobox.setCurrentIndex(index)

    def refresh_timeout(self):
        timeout = self.serial.timeout()
        self.ui.timeout_line_edit.setText(self.strip_alpha(timeout))

    def refresh_ranging(self):
        mode = self.serial.twrmode()
        self.ui.ranging_checkbox.setChecked(bool(self.strtoint(mode)))

    def refresh_leds(self):
        mode = self.serial.ledmode()
        self.ui.led_checkbox.setChecked(not bool(self.strtoint(mode)))

    def refresh_sfd(self):
        mode = self.serial.sfd()
        self.ui.sfd_checkbox.setChecked(bool(self.strtoint(mode)))

    def refresh_amplifiers(self):
        mode = self.serial.pwramp()
        self.ui.extern_amp_combobox.setCurrentIndex(self.strtoint(mode))

    def refresh_antenna(self):
        status = self.serial.status()
        status = BelugaStatus(status)
        self.ui.antenna_checkbox.setChecked(status.secondary_antenna)

    def refresh_datarate(self):
        rate = self.serial.datarate()
        self.ui.uwb_datarate_combobox.setCurrentIndex(self.strtoint(rate))

    def refresh_pulserate(self):
        rate = self.serial.pulserate()
        self.ui.uwb_pulserate_combobox.setCurrentIndex(self.strtoint(rate))

    def refresh_phr(self):
        mode = self.serial.phr()
        self.ui.phr_checkbox.setChecked(bool(self.strtoint(mode)))

    def refresh_preamble(self):
        preamble = self.serial.preamble()
        self.ui.uwb_preamble_combobox.setCurrentIndex(self.preamble_to_index(preamble))

    def refresh_pac(self):
        pac = self.serial.pac()
        self.ui.uwb_pac_combobox.setCurrentIndex(self.strtoint(pac))

    def refresh_pan(self):
        pan = self.serial.panid()
        self.ui.pan_id_text_edit.setText(self.extract_hex(pan))

    def refresh_evict(self):
        if self.ui.eviction_combobox.support:
            mode = self.serial.evict()
            self.ui.eviction_combobox.setCurrentIndex(self.strtoint(mode))

    def refresh_tx_power(self):
        resp = self.serial.txpower()
        power = self.extract_hex(resp)
        if self.check_standard_tx_power(power):
            self.ui.uwb_txpower_combobox.setCurrentIndex(int(power == "1F1F1F1F"))
        else:
            power = int(power, base=16)
            self.ui.boost_norm_coarse_gain.update_power(power)
            self.ui.boost_norm_fine_gain.update_power(power)
            self.ui.boostp500_coarse_gain.update_power(power)
            self.ui.boostp500_fine_gain.update_power(power)
            self.ui.boostp250_coarse_gain.update_power(power)
            self.ui.boostp250_fine_gain.update_power(power)
            self.ui.boostp125_coarse_gain.update_power(power)
            self.ui.boostp125_fine_gain.update_power(power)
        self.ui.connect_status.setText(resp)

    def update_ble(self):
        status = self.serial.status()
        status = BelugaStatus(status)
        if status.ble:
            resp = self.serial.stop_ble()
        else:
            resp = self.serial.start_ble()
        self.refresh_ble()
        self.ui.connect_status.setText(resp)

    def update_uwb(self):
        status = self.serial.status()
        status = BelugaStatus(status)
        if status.uwb:
            resp = self.serial.stop_uwb()
        else:
            resp = self.serial.start_uwb()
        self.refresh_uwb()
        self.ui.connect_status.setText(resp)

    def update_node_id(self, id_: str):
        resp = self.serial.id(id_)
        self.refresh_id()
        self.ui.connect_status.setText(resp)

    def update_bootmode(self, mode: int):
        resp = self.serial.bootmode(mode)
        self.refresh_bootmode()
        self.ui.connect_status.setText(resp)

    def update_rate(self, rate: str):
        resp = self.serial.rate(rate)
        self.refresh_rate()
        self.ui.connect_status.setText(resp)

    def update_channel(self, index: int):
        index += 1
        if index == 6:
            index = 7
        resp = self.serial.channel(index)
        self.refresh_channel()
        self.ui.connect_status.setText(resp)

    def update_timeout(self, timeout: str):
        resp = self.serial.timeout(timeout)
        self.refresh_timeout()
        self.ui.connect_status.setText(resp)

    def update_ranging(self, state: bool):
        resp = self.serial.twrmode(int(state))
        self.refresh_ranging()
        self.ui.connect_status.setText(resp)

    def update_leds(self, state: bool):
        resp = self.serial.ledmode(int(not state))
        self.refresh_leds()
        self.ui.connect_status.setText(resp)

    def update_sfd(self, state: bool):
        resp = self.serial.sfd(int(state))
        self.refresh_sfd()
        self.ui.connect_status.setText(resp)

    def update_amplifiers(self, index: int):
        resp = self.serial.pwramp(index)
        self.refresh_amplifiers()
        self.ui.connect_status.setText(resp)

    def update_antenna(self, state: bool):
        resp = self.serial.antenna(int(state) + 1)
        self.refresh_antenna()
        self.ui.connect_status.setText(resp)

    def update_datarate(self, index: int):
        resp = self.serial.datarate(index)
        self.refresh_datarate()
        self.ui.connect_status.setText(resp)

    def update_pulserate(self, index: int):
        resp = self.serial.pulserate(index)
        self.refresh_pulserate()
        self.ui.connect_status.setText(resp)

    def update_phr(self, state: bool):
        resp = self.serial.phr(int(state))
        self.refresh_phr()
        self.ui.connect_status.setText(resp)

    def update_preamble(self, index: int):
        if index < 5:
            preamble = 2 ** (index + 6)
        elif index == 5:
            preamble = 1536
        else:
            preamble = 2 ** (index + 5)
        resp = self.serial.preamble(preamble)
        self.refresh_preamble()
        self.ui.connect_status.setText(resp)

    def update_pac(self, index: int):
        resp = self.serial.pac(index)
        self.refresh_pac()
        self.ui.connect_status.setText(resp)

    def update_pan(self, id_: str):
        resp = self.serial.panid(f"0x{id_}")
        self.refresh_pan()
        self.ui.connect_status.setText(resp)

    def update_evict(self, index: int):
        resp = self.serial.evict(index)
        self.refresh_evict()
        self.ui.connect_status.setText(resp)

    def update_simple_power(self, index: int):
        resp = self.serial.txpower(f"{index}")
        self.refresh_tx_power()
        self.ui.connect_status.setText(resp)

    def update_complex_power(self):
        self.serial.txpower(
            f"{self.ui.boost_norm_coarse_gain.stage} "
            f"{self.ui.boost_norm_coarse_gain.currentIndex()} "
            f"{int(self.ui.boost_norm_fine_gain.value() / self.ui.boost_norm_fine_gain.singleStep())}")
        self.serial.txpower(
            f"{self.ui.boostp500_coarse_gain.stage} "
            f"{self.ui.boostp500_coarse_gain.currentIndex()} "
            f"{int(self.ui.boostp500_fine_gain.value() / self.ui.boostp500_fine_gain.singleStep())}")
        self.serial.txpower(
            f"{self.ui.boostp250_coarse_gain.stage} "
            f"{self.ui.boostp250_coarse_gain.currentIndex()} "
            f"{int(self.ui.boostp250_fine_gain.value() / self.ui.boostp250_fine_gain.singleStep())}")
        self.serial.txpower(
            f"{self.ui.boostp125_coarse_gain.stage} "
            f"{self.ui.boostp125_coarse_gain.currentIndex()} "
            f"{int(self.ui.boostp125_fine_gain.value() / self.ui.boostp125_fine_gain.singleStep())}")
        self.refresh_tx_power()

    class RebootRunnable(QRunnable):
        def __init__(self, serial: BelugaSerial, callback, widget):
            super().__init__()
            self.serial = serial
            self.update_func = callback
            self.widget = widget

        def run(self):
            self.serial.reboot()
            self.widget.rebooting = False
            self.update_func()

    def reboot(self):
        self.ui.widget_2.rebooting = True
        pool = QThreadPool.globalInstance()
        runnable = self.RebootRunnable(self.serial, self._gather_beluga_data, self.ui.widget_2)
        pool.start(runnable)

    @staticmethod
    def strtoint(response: str) -> int:
        return int("".join([c for c in response if c.isdigit()]))

    @staticmethod
    def extract_hex(response: str) -> str:
        index = response.find("0x")
        if index == -1:
            raise ValueError("Hex number not present")
        return response[index + 2:].split()[0]

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

    @staticmethod
    def check_standard_tx_power(response: str) -> bool:
        return response == "0E080222" or response == "1F1F1F1F"

    def init_uwb_power(self):
        power = self.serial.txpower()
        power = self.extract_hex(power)
        if self.check_standard_tx_power(power):
            self.ui.uwb_txpower_combobox.setCurrentIndex(int(power == "1F1F1F1F"))
            self.ui.uwb_tx_power_checkbox.setChecked(True)
            self.ui.uwb_txpower_combobox.refresh_advanced_power()
        else:
            self.ui.uwb_tx_power_checkbox.setChecked(False)
            power = int(power, base=16)
            self.ui.boost_norm_coarse_gain.update_power(power)
            self.ui.boost_norm_fine_gain.update_power(power)
            self.ui.boostp500_coarse_gain.update_power(power)
            self.ui.boostp500_fine_gain.update_power(power)
            self.ui.boostp250_coarse_gain.update_power(power)
            self.ui.boostp250_fine_gain.update_power(power)
            self.ui.boostp125_coarse_gain.update_power(power)
            self.ui.boostp125_fine_gain.update_power(power)

    def _gather_beluga_data(self) -> Tuple[bool, str]:
        # See if we can get the ID
        try:
            id_ = self.serial.id()
        except Exception:
            return False, "Unable to communicate with port"
        if id_ == 'Response timed out':
            return False, "Unable to communicate with port"
        self.ui.node_id_line_edit.setText(self.strip_alpha(id_))

        status = BelugaStatus(self.serial.status())

        self.refresh_bootmode()
        self.refresh_rate()
        self.refresh_channel()
        self.refresh_timeout()
        self.refresh_ranging()
        self.refresh_leds()
        self.refresh_sfd()

        if status.external_ble_amp and status.external_uwb_amp:
            self.refresh_amplifiers()
            self.ui.extern_amp_combobox.supported(True)
        else:
            self.ui.extern_amp_combobox.supported(False)

        self.ui.antenna_checkbox.supported(status.secondary_antenna_support)
        self.ui.antenna_checkbox.setChecked(status.secondary_antenna)

        self.refresh_datarate()
        self.refresh_pulserate()
        self.refresh_phr()
        self.refresh_preamble()
        self.refresh_pac()
        self.refresh_pan()

        self.ui.eviction_combobox.supported(status.dynamic_eviction_scheme_support)
        self.refresh_evict()
        self.init_uwb_power()

        self.ui.ble_button.set_ble_state(status.ble)
        self.ui.uwb_button.set_uwb_state(status.uwb)

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

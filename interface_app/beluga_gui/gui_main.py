import sys
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QThread, QObject, QThreadPool, QRunnable, QTimer
from typing import Optional, Callable, Iterable, Tuple
from .beluga_gui import Ui_BelugaGUI
from .widgets import BelugaStatusLabel
from .dialogs import DataGatheringDialog, CaptureProgress, messages
from beluga_serial import BelugaSerial, BelugaSerialAttr, BelugaStatus, unpack_beluga_version
from copy import deepcopy
import time
from serial import Serial, SerialException
import math
import signal
from pathlib import Path
import json


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

    class CaptureData:
        def __init__(self, parent, save_file, samples, timeout):
            self._capture_progress = CaptureProgress(parent, samples, timeout)
            self._file: str = save_file
            self._captured_data = []
            self._max_length = samples
            self._dropped_exchanges = {}

        @staticmethod
        def make_header(header: str, length: int):
            if len(header) >= length:
                return header
            padding_len = length - len(header) - 2
            left = padding_len // 2
            right = padding_len - left
            return f"{'-' * left} {header} {'-' * right}\n"

        def _save_normal(self, current_configs: dict[str, str | int | bool]):
            with open(self._file, 'w') as f:
                f.write(self.make_header("Configurations", 80))
                for key, item in current_configs.items():
                    f.write(f"{key}: {item}\n")
                f.write("SS-TWR -> 1 TX and 1 RX per exchange, DS-TWR -> 2 TX and 2 RX per exchange\n")
                f.write(self.make_header("Dropped Exchange Stats", 80))
                f.write("Key: 0: TX Poll, 1: RX Response, 2: TX Final, 3: RX Report\n")
                f.write("Node ID,UWB Exchange Stage,Drops\n")
                for id_, stages in self._dropped_exchanges.items():
                    for stage, drops in stages.items():
                        if drops:
                            f.write(f"{id_},{stage},{drops}\n")
                f.write(self.make_header("Captured Data", 80))
                f.write("Node ID,RSSI,Range\n")
                for sample in self._captured_data:
                    for id_ in sample:
                        f.write(f"{id_},{sample[id_]['RSSI']},{sample[id_]['RANGE']}\n")

        def _save_json(self, current_configs):
            json_dict = {
                "configurations": current_configs,
                "drops": self._dropped_exchanges,
            }

            samples: dict[str, list[dict[str, int | float]]] = {}

            for sample in self._captured_data:
                for id_ in sample:
                    if str(id_) in samples:
                        samples[str(id_)].append({"RSSI": sample[id_]['RSSI'], "RANGE": sample[id_]['RANGE'],
                                                  "UWB_DIAGNOSTICS": sample[id_]["DIAGNOSTICS"],
                                                  "EVENTS": sample[id_]["EVENTS"]})
                    else:
                        samples[str(id_)] = [{"RSSI": sample[id_]['RSSI'], "RANGE": sample[id_]['RANGE'],
                                              "UWB_DIAGNOSTICS": sample[id_]["DIAGNOSTICS"],
                                              "EVENTS": sample[id_]["EVENTS"]}]
            json_dict["samples"] = samples

            with open(self._file, 'w') as f:
                json.dump(json_dict, f, indent='\t')

        def save(self, current_configs):
            messages.InfoMessage(self._capture_progress.parent(), message="Data capture complete")
            if Path(self._file).suffix == ".json":
                self._save_json(current_configs)
            else:
                self._save_normal(current_configs)

        def report_drop(self, drop):
            if len(self._captured_data) < self._max_length:
                if drop['ID'] not in self._dropped_exchanges:
                    self._dropped_exchanges[drop['ID']] = {
                        0: {"count": 0,
                            "events": {"PHE": 0, "RSL": 0, "CRCG": 0, "CRCB": 0, "ARFE": 0, "OVER": 0, "SFDTO": 0,
                                       "PTO": 0, "RTO": 0, "TXF": 0, "HPW": 0, "TXW": 0}},
                        1: {"count": 0,
                            "events": {"PHE": 0, "RSL": 0, "CRCG": 0, "CRCB": 0, "ARFE": 0, "OVER": 0, "SFDTO": 0,
                                       "PTO": 0, "RTO": 0, "TXF": 0, "HPW": 0, "TXW": 0}},
                        2: {"count": 0,
                            "events": {"PHE": 0, "RSL": 0, "CRCG": 0, "CRCB": 0, "ARFE": 0, "OVER": 0, "SFDTO": 0,
                                       "PTO": 0, "RTO": 0, "TXF": 0, "HPW": 0, "TXW": 0}},
                        3: {"count": 0,
                            "events": {"PHE": 0, "RSL": 0, "CRCG": 0, "CRCB": 0, "ARFE": 0, "OVER": 0, "SFDTO": 0,
                                       "PTO": 0, "RTO": 0, "TXF": 0, "HPW": 0, "TXW": 0}},
                    }
                self._dropped_exchanges[drop['ID']][drop['STAGE']]["count"] += 1
                for key in self._dropped_exchanges[drop['ID']][drop['STAGE']]["events"]:
                    self._dropped_exchanges[drop['ID']][drop['STAGE']]["events"][key] += drop["COUNTS"][key]

        def capture_sample(self, sample):
            self._captured_data.append(sample)
            self._capture_progress.setValue(len(self._captured_data))

        def done(self):
            done = (len(self._captured_data) >= self._max_length) or self._capture_progress.wasCanceled()
            if done:
                self._capture_progress.stop()
            return done

        @property
        def canceled(self):
            return self._capture_progress.wasCanceled()

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
        self.ui.ble_button.ble_update["bool"].connect(self.ui.ranges_ranging_pushbutton.update_ble_state)
        self.ui.uwb_button.uwb_update["bool"].connect(self.ui.ranges_ranging_pushbutton.update_uwb_state)
        self.ui.ble_button.ble_update["bool"].connect(self.ui.terminal_ranging_btn.update_ble_state)
        self.ui.uwb_button.uwb_update["bool"].connect(self.ui.terminal_ranging_btn.update_uwb_state)
        self.ui.ranges_ranging_pushbutton.pressed.connect(self.toggle_ranging)
        self.ui.terminal_ranging_btn.pressed.connect(self.toggle_ranging)
        self.ui.clear_neighbors.pressed.connect(self.serial.clear)

        self._neighbor_list_update_timer = QTimer()
        self._neighbor_list_update_timer.setInterval(50)
        self._neighbor_list_update_timer.timeout.connect(self.update_neighbor_list)
        self._neighbor_list_update_timer.start()

        self._neighbor_updates_timer = QTimer()
        self._neighbor_updates_timer.setInterval(100)
        self._neighbor_updates_timer.timeout.connect(self.update_neighbors)
        self._neighbor_updates_timer.start()

        self.ui.record_data_button.pressed.connect(self.start_data_recording)
        self._logging = False

        self._status_label = BelugaStatusLabel(self.ui.statusbar)
        self.ui.statusbar.addWidget(self._status_label)

        self._capturing_data = False
        self._data_capture: Optional[BelugaGui.CaptureData] = None

    def run(self):
        signal.signal(signal.SIGINT, self.quit)
        self.window.show()
        ret = self.app.exec_()
        self.serial.close()
        sys.exit(ret)

    def quit(self, sig, frame):
        self.app.quit()

    def start_data_recording(self):
        dlg = DataGatheringDialog(self.window)
        if dlg.exec():
            if not self.ui.ranges_ranging_pushbutton.ranging:
                # Start ranging
                self.ui.ranges_ranging_pushbutton.pressed.emit()
            self._data_capture = self.CaptureData(self.window, dlg.save_file, dlg.samples, dlg.timeout)
            self.serial.register_dropped_uwb_exchange_hook(self._data_capture.report_drop)
            self._capturing_data = True

    def save_captured_data(self):
        if not self._data_capture.canceled:
            configs = {
                "ID": self.ui.node_id_line_edit.text(),
                "Channel": self.ui.channel_combobox.currentText(),
                "DS-TWR": self.ui.ranging_checkbox.isChecked(),
                "SFD": self.ui.sfd_checkbox.isChecked(),
                "External Amps": self.ui.extern_amp_combobox.currentText(),
                "Data rate": self.ui.uwb_datarate_combobox.currentText(),
                "Pulse rate": self.ui.uwb_pulserate_combobox.currentText(),
                "PHR": self.ui.phr_checkbox.isChecked(),
                "Preamble": self.ui.uwb_preamble_combobox.currentText(),
                "PAC": self.ui.uwb_pac_combobox.currentText(),
                "TX Power": self.serial.txpower(),
            }
            self._data_capture.save(configs)
        self._data_capture = None
        self._capturing_data = False

    def record_sample(self, sample):
        if self._capturing_data:
            if self._data_capture.done():
                self._data_capture = None
                self._capturing_data = False
                return
            self._data_capture.capture_sample(sample)
            if self._data_capture.done():
                self.save_captured_data()

    def update_neighbor_list(self):
        update, neighbors = self.serial.get_neighbors()
        if update:
            self.ui.neighbors.update_neighbor_list(neighbors)
            self.ui.distance_graph.update_neighbor_list(neighbors)
            self.ui.rssi_graph.update_neighbor_list(neighbors)
            self.ui.rssi_v_distance.update_neighbor_list(neighbors)

    def update_neighbors(self):
        updates = self.serial.get_ranges()
        if updates:
            self.ui.neighbors.update_neighbors(updates)
            self.ui.distance_graph.update_neighbors(updates)
            self.ui.rssi_graph.update_neighbors(updates)
            self.ui.rssi_v_distance.update_neighbors(updates)
            self.record_sample(updates)

    def toggle_ranging(self):
        if self.ui.ranges_ranging_pushbutton.ranging:
            self.ui.uwb_button.pressed.emit()
            self.ui.ble_button.pressed.emit()
        else:
            if not self.ui.ble_button.ble_running:
                self.ui.ble_button.pressed.emit()
            self.ui.uwb_button.pressed.emit()

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
        id_ = self.strip_alpha(id_)
        self.ui.node_id_line_edit.setText(id_)
        self.ui.ranges_id.update_config(id_)
        self.ui.terminal_node_id.update_config(id_)

    def refresh_bootmode(self):
        mode = self.serial.bootmode()
        mode = self.strtoint(mode)
        self.ui.boot_mode_combobox.setCurrentIndex(mode)

    def refresh_rate(self):
        rate = self.serial.rate()
        rate = self.strip_alpha(rate)
        self.ui.rate_line_edit.setText(rate)
        self.ui.ranges_poll_rate.update_config(rate)
        self.ui.terminal_poll_rate.update_config(rate)

    def refresh_channel(self):
        channel = self.serial.channel()
        index = self.channel_to_index(channel)
        self.ui.channel_combobox.setCurrentIndex(index)
        channel = self.strip_alpha(channel)
        self.ui.ranges_channel.update_config(channel)
        self.ui.terminal_channel.update_config(channel)

    def refresh_timeout(self):
        timeout = self.serial.timeout()
        timeout = self.strip_alpha(timeout)
        self.ui.timeout_line_edit.setText(timeout)
        self.ui.ranges_timeout.update_config(timeout)
        self.ui.terminal_timeout.update_config(timeout)

    def refresh_ranging(self):
        mode = self.serial.twrmode()
        mode = bool(self.strtoint(mode))
        self.ui.ranging_checkbox.setChecked(mode)
        self.ui.ranges_ranging_mode.update_config(mode)
        self.ui.terminal_ranging_mode.update_config(mode)

    def refresh_leds(self):
        mode = self.serial.ledmode()
        self.ui.led_checkbox.setChecked(not bool(self.strtoint(mode)))

    def refresh_sfd(self):
        mode = self.serial.sfd()
        mode = bool(self.strtoint(mode))
        self.ui.sfd_checkbox.setChecked(mode)
        self.ui.ranges_sfd.update_config(mode)
        self.ui.terminal_sfd.update_config(mode)

    def refresh_amplifiers(self):
        mode = self.serial.pwramp()
        mode = self.strtoint(mode)
        self.ui.extern_amp_combobox.setCurrentIndex(mode)
        self.ui.ranges_power_amp.update_config(self.ui.extern_amp_combobox.currentText())
        self.ui.terminal_power_amp.update_config(self.ui.extern_amp_combobox.currentText())

    def refresh_antenna(self):
        status = self.serial.status()
        status = BelugaStatus(status)
        self.ui.antenna_checkbox.setChecked(status.secondary_antenna)

    def refresh_datarate(self):
        rate = self.serial.datarate()
        self.ui.uwb_datarate_combobox.setCurrentIndex(self.strtoint(rate))
        self.ui.ranges_datarate.update_config(self.ui.uwb_datarate_combobox.currentText())
        self.ui.terminal_datarate.update_config(self.ui.uwb_datarate_combobox.currentText())

    def refresh_pulserate(self):
        rate = self.serial.pulserate()
        self.ui.uwb_pulserate_combobox.setCurrentIndex(self.strtoint(rate))
        self.ui.ranges_pulserate.update_config(self.ui.uwb_pulserate_combobox.currentText())
        self.ui.terminal_pulserate.update_config(self.ui.uwb_pulserate_combobox.currentText())

    def refresh_phr(self):
        mode = self.serial.phr()
        mode = bool(self.strtoint(mode))
        self.ui.phr_checkbox.setChecked(mode)
        self.ui.ranges_phr.update_config(mode)
        self.ui.terminal_phr.update_config(mode)

    def refresh_preamble(self):
        preamble = self.serial.preamble()
        self.ui.uwb_preamble_combobox.setCurrentIndex(self.preamble_to_index(preamble))
        self.ui.ranges_preamble.update_config(self.ui.uwb_preamble_combobox.currentText())
        self.ui.terminal_preamble.update_config(self.ui.uwb_preamble_combobox.currentText())

    def refresh_pac(self):
        pac = self.serial.pac()
        self.ui.uwb_pac_combobox.setCurrentIndex(self.strtoint(pac))
        self.ui.ranges_pac.update_config(self.ui.uwb_pac_combobox.currentText())
        self.ui.terminal_pac.update_config(self.ui.uwb_pac_combobox.currentText())

    def refresh_pan(self):
        pan = self.serial.panid()
        pan = self.extract_hex(pan)
        self.ui.pan_id_text_edit.setText(pan)
        self.ui.ranges_pan.update_config(pan)
        self.ui.terminal_pan.update_config(pan)

    def refresh_evict(self):
        if self.ui.eviction_combobox.support:
            mode = self.serial.evict()
            self.ui.eviction_combobox.setCurrentIndex(self.strtoint(mode))
        self.ui.ranges_evict.update_config(self.ui.eviction_combobox.currentText())
        self.ui.terminal_evict.update_config(self.ui.eviction_combobox.currentText())

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
        self.ui.ranges_txpower.update_config(power)
        self.ui.terminal_txpower.update_config(power)

    def refresh_node_version(self):
        resp = self.serial.version()
        version = unpack_beluga_version(resp)
        self._status_label.version = f"{version.major}.{version.minor}.{version.patch}"

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
            time.sleep(0.1)
            self.update_func()
            self.widget.rebooting = False

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
        self.refresh_id()

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
        self.refresh_node_version()

        return True, ""

    def update_connection_state(self):
        if self._connected:
            try:
                self.serial.close()
            except Exception as e:
                messages.ErrorMessage(self.window, "FATAL", f"Unable to close serial connection: {e}")
                sys.exit(1)
            self.ui.uwb_txpower_combobox.disconnecting = True
            self.ui.connect_button.update_connected(False, "")
            self.ui.ranges_ranging_pushbutton.update_connected_state(False)
            self.ui.terminal_ranging_btn.update_connected_state(False)
            self.ui.record_data_button.update_connected_state(False)
            self._status_label.set_connected(False)
            self._connected = False
            self.ui.uwb_txpower_combobox.disconnecting = False
        else:
            port = self.ui.device_combobox.currentText()
            port_: str = port.rsplit(":", 1)[-1].lstrip()
            try:
                self.serial.open_target(port_)
            except FileNotFoundError:
                messages.ErrorMessage(self.window, "Open error", "Unable to open port")
            except ValueError:
                self.serial.close()
                self.ui.connect_button.update_connected(False, "Unable to communicate with port")
            else:
                status, error = self._gather_beluga_data()
                self.ui.connect_button.update_connected(status, error)
                self.ui.ranges_ranging_pushbutton.update_connected_state(status)
                self.ui.terminal_ranging_btn.update_connected_state(status)
                self.ui.record_data_button.update_connected_state(status)
                self._connected = status
                self._status_label.port = port
                self._status_label.set_connected(status)


def main():
    gui = BelugaGui(sys.argv)
    gui.run()


if __name__ == "__main__":
    main()

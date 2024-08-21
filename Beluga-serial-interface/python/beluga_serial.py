import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional, Tuple
import enum
import threading
import time


TARGETS = [
    'CMU Beluga'
]


class BelugaBootMode(enum.IntEnum):
    WIRELESS_OFF = 0
    BLE_ON = 1
    BLE_UWB_ON = 2


class BelugaSerial:
    def __init__(self, baud: int = 115200, timeout: float = 2.0, serial_timeout: float = 0.5):
        targets = self._find_ports(TARGETS)
        if not targets:
            raise FileNotFoundError(f'Unable to find a given target. Valid targets: {TARGETS}')
        for target in TARGETS:
            if target in targets.keys():
                print(f"Connecting to {target}: {targets[target][0]}")
                self._serial = serial.Serial(targets[target][0], baudrate=baud, timeout=serial_timeout)
                break
        self._beluga_states = {
            'led_mode': 0,
            'id': 0,
            'bootmode': 0,
            'rate': 100,
            'channel': 5,
            'timeout': 9000,
            'tx_power': 0,
            'stream': 0,
            'ranging': 0,
        }
        self._serial_lock = threading.Lock()
        self._response: str = ''
        self._response_received = threading.BoundedSemaphore()
        self._timeout = timeout
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._outstream = None
        self._outstream_lock = threading.Lock()

    @staticmethod
    def _find_ports(targets: List[str]) -> Dict[str, List[str]]:
        ret: Dict[str, List[str]] = {}
        ports = list(list_ports.comports())

        for port in ports:
            dev_name = f'{port.manufacturer} {port.product}'
            if dev_name in targets:
                if dev_name in ret.keys():
                    ret[dev_name].append(port.device)
                else:
                    ret[dev_name] = [port.device]
        return ret

    def _get_lines(self) -> List[bytes]:
        self._serial_lock.acquire()
        lines = self._serial.readlines()
        self._serial_lock.release()
        return lines

    def _update_settings(self, lines: List[str]) -> Tuple[int, bool]:
        lines_processed = 0
        settings_processed = 0
        node_uninit = False
        NUM_SETTING_LINES = 11

        for line in lines:
            if settings_processed == NUM_SETTING_LINES:
                break
            if line.startswith('Node On:'):
                lines_processed += 1
                settings_processed += 1
            elif line == 'Flash Configuration:':
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('LED Mode: '):
                if 'Off' in line:
                    self._beluga_states['led_mode'] = 0
                elif 'On' in line:
                    self._beluga_states['led_mode'] = 1
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('!'):
                lines_processed += 1
                if not node_uninit:
                    node_uninit = True
                    settings_processed += 1
            elif line.startswith('Node ID: '):
                self._beluga_states['id'] = int(line[9:])
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('Boot Mode: '):
                self._beluga_states['bootmode'] = 0
                if 'BLE ON' in line:
                    self._beluga_states['bootmode'] = 1
                if 'UWB ON' in line:
                    if self._beluga_states['bootmode'] != 1:
                        raise ValueError(f'Invalid bootmode received: {line}')
                    self._beluga_states['bootmode'] += 1
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('UWB Polling Rate: '):
                rate = line[18:]
                try:
                    self._beluga_states['rate'] = int(rate)
                except ValueError:
                    pass
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('UWB Channel: '):
                channel = line[13:]
                try:
                    self._beluga_states['channel'] = int(channel)
                except ValueError:
                    pass
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('BLE Timeout: '):
                timeout = line[13:]
                try:
                    self._beluga_states['timeout'] = int(timeout)
                except ValueError:
                    pass
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('TX Power: '):
                if 'Max' in line:
                    self._beluga_states['tx_power'] = 1
                else:
                    self._beluga_states['tx_power'] = 0
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('Stream Mode: '):
                mode = line[13:]
                try:
                    self._beluga_states['stream'] = int(mode)
                except ValueError:
                    pass
                lines_processed += 1
                settings_processed += 1
            elif line.startswith('Ranging Mode: '):
                mode = line[13:]
                try:
                    self._beluga_states['ranging'] = int(mode)
                except ValueError:
                    pass
                lines_processed += 1
                settings_processed += 1
        return lines_processed, settings_processed == NUM_SETTING_LINES

    def _write_ranging_batch(self, lines: List[str]) -> int:
        lines_processed = 0
        for line in lines:
            if line == '# ID, RANGE, RSSI, TIMESTAMP':
                lines_processed += 1
            elif line[0].isdigit():
                lines_processed += 1
                # TODO
            else:
                break
        return lines_processed


    def _receive_response(self, response: str):
        self._response = response
        self._response_received.release()

    def _process_lines(self, lines: List[bytes]):
        i = 0
        l = len(lines)
        lines = [line.decode(errors='ignore').strip() for line in lines]

        while i < l:
            if not lines[i]:
                # Empty line
                i += 1
            elif lines[i][0].isdigit() or lines[i] == '# ID, RANGE, RSSI, TIMESTAMP':
                processed = self._write_ranging_batch(lines)
                i += processed
            elif lines[i].startswith('Node On'):
                while True:
                    processed, all_settings = self._update_settings(lines[i:])
                    if all_settings:
                        i += processed
                        break
                    else:
                        new_lines = [line.decode().strip() for line in self._get_lines()]
                        lines += new_lines
                        l = len(lines)
            else:
                self._receive_response(lines[i])
                i += 1

    def _read_serial(self):
        self._response_received.acquire(blocking=False)
        while not self._stop.is_set():
            lines = self._get_lines()
            if lines:
                self._process_lines(lines)
            time.sleep(0.5)

    def _send_command(self, command: bytes) -> str:
        self._serial_lock.acquire()
        self._serial.write(command)
        self._serial_lock.release()
        if not self._response_received.acquire(timeout=self._timeout):
            return 'Response timed out'
        ret = self._response
        self._response = None
        return ret

    def start_uwb(self) -> str:
        ret = self._send_command(b'AT+STARTUWB\r\n')
        return ret

    def stop_uwb(self) -> str:
        ret = self._send_command(b'AT+STOPUWB\r\n')
        return ret

    def start_ble(self) -> str:
        ret = self._send_command(b'AT+STARTBLE\r\n')
        return ret

    def stop_ble(self) -> str:
        ret = self._send_command(b'AT+STOPBLE\r\n')
        return ret

    def id(self, new_id: int) -> str:
        command = f'AT+ID {new_id}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def bootmode(self, boot_mode: BelugaBootMode) -> str:
        if boot_mode not in BelugaBootMode:
            return 'Invalid bootmode parameter'
        command = f'AT+BOOTMODE {int(boot_mode)}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def rate(self, rate: int) -> str:
        command = f'AT+RATE {rate}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def channel(self, channel: int) -> str:
        command = f'AT+CHANNEL {channel}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def reset(self) -> str:
        ret = self._send_command(b'AT+RESET\r\n')
        return ret

    def timeout(self, timeout: int) -> str:
        command = f'AT+TIMEOUT {timeout}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def tx_power(self, max_power: int) -> str:
        command = f'AT+TXPOWER {max_power}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def stream_mode(self, updates_only: int) -> str:
        command = f'AT+STREAMMODE {updates_only}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def twr_mode(self, mode: int) -> str:
        command = f'AT+TWRMODE {mode}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def led_mode(self, led_mode: int) -> str:
        command = f'AT+LEDMODE {led_mode}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def reboot(self) -> str:
        ret = self._send_command(b'AT+REBOOT\r\n')
        return ret

    def pwr_amp(self, enable_pwr_amp: int) -> str:
        command = f'AT+PWRAMP {enable_pwr_amp}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def antenna(self, antenna: int) -> str:
        command = f'AT+ANTENNA {antenna}\r\n'
        ret = self._send_command(command.encode())
        return ret

    def start(self):
        if self._rx_thread is not None:
            raise RuntimeError('Please stop before restarting')
        self._rx_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._rx_thread.start()

    def stop(self):
        if self._rx_thread is None:
            return
        self._stop.set()
        self._rx_thread.join()

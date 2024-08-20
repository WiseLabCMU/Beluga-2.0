import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional
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
    def __init__(self, baud: int = 115200, timeout: float = 2.0):
        targets = self._find_ports(TARGETS)
        if not targets:
            raise FileNotFoundError(f'Unable to find a given target. Valid targets: {TARGETS}')
        for target in TARGETS:
            if target in targets.keys():
                print(f"Connecting to {target}: {targets[target][0]}")
                self._serial = serial.Serial(targets[target][0], baudrate=baud)
                break
        self._beluga_states = {
            'led_mode': -1,
            'id': 0,
            'bootmode': -1,
            'rate': -1,
            'channel': -1,
            'timeout': -1,
            'tx_power': -1,
            'stream': -1,
            'ranging': -1
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

    def _update_settings(self, lines: List[bytes]):
        pass

    def _write_ranging_batch(self, lines: List[bytes]):
        pass

    def _receive_response(self, response: bytes):
        self._response = response.decode('utf-8')
        self._response_received.release()

    def _parse_lines(self, lines: List[bytes]):
        pass

    def _read_serial(self):
        while not self._stop.is_set():
            self._serial_lock.acquire()
            lines = self._serial.readlines()
            self._serial_lock.release()
            if lines:
                self._parse_lines(lines)
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

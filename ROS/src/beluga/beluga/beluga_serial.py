from io import TextIOWrapper
import sys
import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional, Tuple, Union, TextIO
import threading
import time
import json

TARGETS = [
    'CMU Beluga',
    'SEGGER J-Link'
]


class BelugaSerial:
    def __init__(self, baud: int = 115200, timeout: float = 2.0, serial_timeout: float = 0.5, max_lines_read: int = 16):
        targets = self._find_ports(TARGETS)
        if not targets:
            raise FileNotFoundError(f'Unable to find a given target. Valid targets: {TARGETS}')
        for target in TARGETS:
            if target in targets.keys():
                print(f"Connecting to {target}: {targets[target][0]}")
                self._serial = serial.Serial(targets[target][0], baudrate=baud, timeout=serial_timeout)
                break
        self._serial_lock = threading.Lock()
        self._response: str = ''
        self._response_received = threading.BoundedSemaphore()
        self._response_received.acquire()
        self._timeout = timeout
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._outstream: Union[TextIOWrapper, TextIO] = sys.stdout
        self._outstream_lock = threading.Lock()
        self._neighbor_list: Dict[int, Dict[str, Union[int, float]]] = {}
        self._read_max_lines: int = max_lines_read
        self.start()

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
        lines = []
        self._serial_lock.acquire()
        for _ in range(self._read_max_lines):
            line = self._serial.readline()
            if not line:
                break
            lines.append(line)
        self._serial_lock.release()
        return lines

    def get_neighbors_list(self) -> Dict[int, Dict[str, Union[int, str]]]:
        return self._neighbor_list

    @staticmethod
    def _parse_entry(line: str) -> Optional[Dict[str, Dict[str, Union[int, float]]]]:
        entries = line.split(',')
        entry = None
        try:
            entry = {'ID': int(entries[0]), 'RANGE': float(entries[1]), 'RSSI': int(entries[2]), 'TIMESTAMP': int(entries[3])}
        except Exception as e:
            print(str(e))
            # Could not parse entry
            pass
        return entry

    def _write_ranging_batch(self, lines: List[str]) -> int:
        lines_processed = 0
        for line in lines:
            try:
                entry = json.loads(line)
                lines_processed += 1
                self._neighbor_list[entry['ID']] = entry
            except json.JSONDecodeError:
                if line == '# ID, RANGE, RSSI, TIMESTAMP':
                    lines_processed += 1
                elif line[0].isdigit():
                    lines_processed += 1
                    self._outstream_lock.acquire()
                    self._outstream.write(line)
                    self._outstream.flush()
                    self._outstream_lock.release()
                    entry = self._parse_entry(line)
                    if entry is not None:
                        self._neighbor_list[entry['ID']] = entry
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
            if lines[i].startswith('{') or lines[i][0].isdigit() or lines[i] == '# ID, RANGE, RSSI, TIMESTAMP':
                processed = self._write_ranging_batch(lines[i:])
                i += processed
                continue
            elif lines[i].endswith('OK'):
                self._receive_response(lines[i])
            elif lines[i].startswith('rm'):
                uuid = int(lines[i].lstrip('rm '))
                if uuid in self._neighbor_list.keys():
                    del self._neighbor_list[uuid]
            i += 1

    def _read_serial(self):
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

    def id(self, new_id: Optional[int] = None) -> str:
        if new_id is not None:
            command = f'AT+ID {new_id}\r\n'.encode()
        else:
            command = b'AT+ID\r\n'
        ret = self._send_command(command)
        return ret

    def bootmode(self, boot_mode: Optional[int] = None) -> str:
        if boot_mode is not None:
            command = f'AT+BOOTMODE {int(boot_mode)}\r\n'.encode()
        else:
            command = b'AT+BOOTMODE\r\n'
        ret = self._send_command(command)
        return ret

    def rate(self, rate: Optional[int] = None) -> str:
        if rate is not None:
            command = f'AT+RATE {rate}\r\n'.encode()
        else:
            command = b'AT+RATE\r\n'
        ret = self._send_command(command)
        return ret

    def channel(self, channel: Optional[int] = None) -> str:
        if channel is not None:
            command = f'AT+CHANNEL {channel}\r\n'.encode()
        else:
            command = b'AT+CHANNEL\r\n'
        ret = self._send_command(command)
        return ret

    def reset(self) -> str:
        ret = self._send_command(b'AT+RESET\r\n')
        return ret

    def timeout(self, timeout: Optional[int] = None) -> str:
        if timeout is not None:
            command = f'AT+TIMEOUT {timeout}\r\n'.encode()
        else:
            command = b'AT+TIMEOUT\r\n'
        ret = self._send_command(command)
        return ret

    def tx_power(self, max_power: Optional[int] = None) -> str:
        if max_power is not None:
            command = f'AT+TXPOWER {max_power}\r\n'.encode()
        else:
            command = b'AT+TXPOWER\r\n'
        ret = self._send_command(command)
        return ret

    def stream_mode(self, updates_only: Optional[int] = None) -> str:
        if updates_only is not None:
            command = f'AT+STREAMMODE {updates_only}\r\n'.encode()
        else:
            command = b'AT+STREAMMODE\r\n'
        ret = self._send_command(command)
        return ret

    def twr_mode(self, mode: Optional[int] = None) -> str:
        if mode is not None:
            command = f'AT+TWRMODE {mode}\r\n'.encode()
        else:
            command = b'AT+TWRMODE\r\n'
        ret = self._send_command(command)
        return ret

    def led_mode(self, led_mode: Optional[int] = None) -> str:
        if led_mode is not None:
            command = f'AT+LEDMODE {led_mode}\r\n'.encode()
        else:
            command = b'AT+LEDMODE\r\n'
        ret = self._send_command(command)
        return ret

    def reboot(self) -> str:
        ret = self._send_command(b'AT+REBOOT\r\n')
        return ret

    def pwr_amp(self, enable_pwr_amp: Optional[int] = None) -> str:
        if enable_pwr_amp is not None:
            command = f'AT+PWRAMP {enable_pwr_amp}\r\n'.encode()
        else:
            command = b'AT+PWRAMP\r\n'
        ret = self._send_command(command)
        return ret

    def antenna(self, antenna: Optional[int] = None) -> str:
        if antenna is not None:
            command = f'AT+ANTENNA {antenna}\r\n'.encode()
        else:
            command = b'AT+ANTENNA\r\n'
        ret = self._send_command(command)
        return ret

    def time(self) -> str:
        command = b'AT+TIME\r\n'
        ret = self._send_command(command)
        return ret

    def format(self, json_format: Optional[int] = None) -> str:
        if json_format is not None:
            command = f'AT+FORMAT {json_format}\r\n'.encode()
        else:
            command = b'AT+FORMAT\r\n'
        ret = self._send_command(command)
        return ret

    def deepsleep(self) -> str:
        command = b'AT+DEEPSLEEP\r\n'
        ret = self._send_command(command)
        return ret

    def datarate(self, rate: Optional[int] = None) -> str:
        if rate is not None:
            command = f'AT+DATARATE {rate}\r\n'.encode()
        else:
            command = b'AT+DATARATE\r\n'
        ret = self._send_command(command)
        return ret

    def preamble(self, length: Optional[int] = None) -> str:
        if length is not None:
            command = f'AT+PREAMBLE {length}\r\n'.encode()
        else:
            command = b'AT+PREAMBLE\r\n'
        ret = self._send_command(command)
        return ret

    def pulserate(self, rate: Optional[int] = None) -> str:
        if rate is not None:
            command = f'AT+PULSERATE {rate}\r\n'.encode()
        else:
            command = b'AT+PULSERATE\r\n'
        ret = self._send_command(command)
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

    def set_outstream(self, stream: Union[TextIOWrapper, TextIO]):
        self._outstream_lock.acquire()
        if self._outstream != sys.stdout:
            self._outstream.close()
        self._outstream = stream
        self._outstream_lock.release()


def main():
    beluga = BelugaSerial()
    ret = beluga.format(1)
    ret = beluga.format()
    print(ret)

    while True:
        pass


if __name__ == '__main__':
    main()

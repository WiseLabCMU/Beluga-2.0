from io import TextIOWrapper
import sys
import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional, Union, TextIO, Callable, Any, Tuple
import multiprocessing as mp
import multiprocessing.queues as mp_queues
import queue
import time
import json
import re

TARGETS = [
    'CMU Beluga',
    'SEGGER J-Link'
]


class BelugaEntryError(Exception):
    def __init__(self, msg: str, header: bool = False):
        self.header = header
        super().__init__(msg)


class BelugaNeighborListEntry:
    def __init__(self, line: str):
        self._id = 0
        self._range = 0
        self._rssi = 0
        self._time = 0
        self._updated = False

        ret = self.update_entry(line)
        if not ret:
            raise BelugaEntryError('Line is a header', True)
        return

    @property
    def id(self) -> int:
        return self._id

    @property
    def range(self) -> float:
        return self._range

    @property
    def rssi(self) -> int:
        return self._rssi

    @property
    def time(self) -> int:
        return self._time

    @property
    def updated(self) -> bool:
        return self._updated

    @updated.setter
    def updated(self, update: bool) -> None:
        self._updated = update
        return

    @staticmethod
    def _parse_entry(line):
        entries = line.split(',')
        try:
            entry = {'ID': int(entries[0]), 'RANGE': float(entries[1]), 'RSSI': int(entries[2]),
                     'TIMESTAMP': int(entries[3])}
        except Exception as e:
            raise BelugaEntryError(f'Incomplete entry: {e}')
        return entry

    def update_entry(self, line: str) -> bool:
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            if line == '# ID, RANGE, RSSI, TIMESTAMP':
                return False
            entry = self._parse_entry(line)
        self._id = entry['ID']
        self._range = entry['RANGE']
        self._rssi = entry['RSSI']
        self._time = entry['TIMESTAMP']
        self._updated = True
        return True


class BelugaNeighborsList:
    def __init__(self):
        self._list: Dict[str, BelugaNeighborListEntry] = {}
        self._neighbors_update: bool = False
        self._range_update: bool = False
        return

    @staticmethod
    def _parse_id(line: str) -> str:
        return re.sub('[^0-9]', "", line.split(',', 1)[0])

    def update(self, line: str) -> None:
        _id = self._parse_id(line)
        if not _id:
            return
        if _id not in self._list.keys():
            self._list[_id] = BelugaNeighborListEntry(line)
            self._range_update = True
            self._neighbors_update = True
            return
        if self._list[_id].update_entry(line):
            self._range_update = True
        return

    def remove_neighbor(self, line: str):
        _id = self._parse_id(line)
        if not _id:
            return
        if _id in self._list.keys():
            del self._list[_id]
            self._neighbors_update = True
        return

    def get_updates(self) -> Dict[int, Dict[str, Union[int, float]]]:
        ret = {}
        for x in self._list.values():
            if x.updated:
                ret[x.id] = {"RANGE": x.range, "RSSI": x.rssi, "TIMESTAMP": x.time}
                x.updated = False
        self._range_update = False
        return ret

    def get_list(self) -> Dict[int, Dict[str, Union[int, float]]]:
        ret = {}
        for x in self._list.values():
            ret[x.id] = {"RANGE": x.range, "RSSI": x.rssi, "TIMESTAMP": x.time}
        self._neighbors_update = False
        return ret

    @property
    def neighbors_update(self) -> bool:
        return self._neighbors_update

    @property
    def range_update(self) -> bool:
        return self._range_update


class BelugaQueue(mp_queues.Queue):
    def __init__(self, maxsize: int = 1):
        super().__init__(maxsize=maxsize, ctx=mp.get_context())

    def put(self, obj, block: bool = True, timeout: Optional[float] = None) -> None:
        try:
            super().put(obj, block, timeout)
        except queue.Full:
            # Discard oldest item
            self.get_nowait()
            super().put(obj, block, timeout)
        return


class BelugaSerial:
    # TODO: Discuss how we want to handle case where USB communication fails. Do we want to log the error and try to reconnect? Do we want to terminate the node?
    def __init__(self, baud: int = 115200, timeout: float = 2.0, serial_timeout: float = 0.1, max_lines_read: int = 16,
                 port: Optional[str] = None, logger_func: Optional[Callable[[Any], None]] = None):

        self._logger = logger_func

        if port is None:
            targets = self._find_ports(TARGETS)
            self._serial = None
            if not targets:
                raise FileNotFoundError(f'Unable to find a given target. Valid targets: {TARGETS}')
            for target in TARGETS:
                if target in targets.keys():
                    for port in targets[target]:
                        try:
                            self._log(f"Trying to connect to {target}: {port}")
                            self._serial = serial.Serial(port=port, baudrate=baud, timeout=serial_timeout,
                                                         exclusive=True)
                            self._log(f'Connected to {target}: {port}')
                            break
                        except serial.SerialException:
                            self._log(f'{port} is busy')
                            pass
                    break
            if self._serial is None:
                raise FileNotFoundError(f'Unable to find a given target. Valid targets: {TARGETS}')
        else:
            self._serial = serial.Serial(port=port, baudrate=baud, timeout=serial_timeout, exclusive=True)

        self._read_max_lines: int = max_lines_read
        self._timeout: float = timeout
        self._neighbors = BelugaNeighborsList()

        self._rx_task: Optional[mp.Process] = None
        self._batch_queue: BelugaQueue = BelugaQueue(5)

        self._processing_task: Optional[mp.Process] = None
        self._response_q: BelugaQueue = BelugaQueue()
        self._ranges_queue: BelugaQueue = BelugaQueue()
        self._neighbors_queue: BelugaQueue = BelugaQueue()
        self._command_sent: mp.Event = mp.Event()
        self._reboot_done: mp.Event = mp.Event()

    def _log(self, s):
        if self._logger is not None:
            self._logger(s)

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

    def _write_ranging_batch(self, lines: List[str]) -> int:
        lines_processed = 0
        for line in lines:
            try:
                self._neighbors.update(line)
                lines_processed += 1
            except BelugaEntryError as e:
                if e.header:
                    lines_processed += 1
                else:
                    return lines_processed
        return lines_processed

    def _process_lines(self):
        rebooting = False
        while True:
            lines = self._batch_queue.get()
            i = 0
            l = len(lines)
            lines = [line.decode(errors='ignore').strip() for line in lines]

            while i < l:
                if not lines[i]:
                    if rebooting:
                        rebooting = False
                        self._reboot_done.set()
                    i += 1
                    continue
                if lines[i].startswith('{') or lines[i][0].isdigit() or lines[i] == '# ID, RANGE, RSSI, TIMESTAMP':
                    processed = self._write_ranging_batch(lines[i:])
                    i += processed
                    if processed == 0:
                        # Incomplete line
                        i += 1
                    continue
                if lines[i].startswith('rm '):
                    self._neighbors.remove_neighbor(lines[i])
                    i += 1
                if self._command_sent.is_set() and i < l:
                    self._command_sent.clear()
                    self._response_q.put(lines[i])
                    i += 1
                else:
                    # Probably boot info, wait until settings are printed to resume
                    rebooting = True
                    i += 1

            if self._neighbors.range_update:
                self._ranges_queue.put(self._neighbors.get_updates(), block=False)
            if self._neighbors.neighbors_update:
                self._neighbors_queue.put(self._neighbors.get_list(), block=False)

    def _get_lines(self) -> List[bytes]:
        lines = []
        for _ in range(self._read_max_lines):
            line = self._serial.readline()
            if not line:
                break
            lines.append(line)
        return lines

    def _read_serial(self):
        while True:
            lines = self._get_lines()
            if lines:
                self._batch_queue.put(lines, block=False)

    def _send_command(self, command: bytes) -> str:
        self._serial.write(command)
        try:
            self._command_sent.set()
            ret = self._response_q.get(timeout=self._timeout)
        except queue.Empty:
            ret = 'Response timed out'
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
        self._reboot_done.wait()
        self._reboot_done.clear()
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
        if self._rx_task is not None or self._processing_task is not None:
            raise RuntimeError('Please stop before restarting')
        self._processing_task = mp.Process(target=self._process_lines)
        self._rx_task = mp.Process(target=self._read_serial)

        self._processing_task.start()
        self._rx_task.start()

    def stop(self):
        if self._rx_task is not None:
            self._rx_task.kill()
            while self._rx_task.is_alive():
                time.sleep(0.1)
            self._rx_task.close()
        if self._processing_task is not None:
            self._processing_task.kill()
            while self._processing_task.is_alive():
                pass
            self._processing_task.close()
        return

    def get_neighbors(self) -> Tuple[bool, Dict[int, Dict[str, Union[int, float]]]]:
        # Needed to indicate if the queue is just empty or if the only neighbor got removed
        update = True
        ret = {}
        try:
            ret = self._neighbors_queue.get_nowait()
        except queue.Empty:
            update = False
        return update, ret

    def get_ranges(self) -> Dict[int, Dict[str, Union[int, float]]]:
        ret = {}
        try:
            ret = self._ranges_queue.get_nowait()
        except queue.Empty:
            pass
        return ret


def main():
    import datetime as dt
    beluga = BelugaSerial()
    beluga.start()
    beluga.reboot()
    beluga.stream_mode(True)
    last_tr = dt.datetime.now()

    try:
        while True:
            ranges = beluga.get_ranges()
            if ranges:
                print(f'Ranges {ranges}')
            update, neighbors = beluga.get_neighbors()
            if update:
                print(f'Neighbors: {neighbors}')

            elapsed = dt.datetime.now() - last_tr
            if elapsed.microseconds > 100000:
                ret = beluga.time()
                print(ret)
                last_tr = dt.datetime.now()
    except KeyboardInterrupt:
        beluga.stop()


if __name__ == '__main__':
    main()

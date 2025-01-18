from io import TextIOWrapper
import sys
import os
import signal
import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional, Union, TextIO, Callable, Any, Tuple
import multiprocessing as mp
import multiprocessing.queues as mp_queues
import queue
import time
import itertools
from beluga.beluga_frame import BelugaFrame, FrameType

TARGETS = [
    'CMU Beluga',
    'SEGGER J-Link'
]


class BelugaEntryError(Exception):
    def __init__(self, msg: str, header: bool = False):
        self.header = header
        super().__init__(msg)


class BelugaNeighbor:
    def __init__(self, neighbor: dict):
        self._id: int = neighbor["ID"]
        self._range: float = 0.0
        self._rssi: int = 0
        self._time: int = 0
        self._exchange_id: int = 0
        self._updated = False
        self.update(neighbor)

    def __iter__(self):
        return iter([("RANGE", self._range), ("RSSI", self._rssi), ("TIMESTAMP", self._time), ("EXCHANGE", self._exchange_id)])

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
    def exchange(self) -> int:
        return self._exchange_id

    @property
    def updated(self) -> bool:
        return self._updated

    @updated.setter
    def updated(self, update: bool):
        self._updated = update

    def update(self, neighbor: dict):
        self._range = neighbor["RANGE"]
        self._rssi = neighbor["RSSI"]
        self._time = neighbor["TIMESTAMP"]
        if "EXCHANGE" in neighbor.keys():
            self._exchange_id = neighbor["EXCHANGE"]
        self._updated = True


class BelugaNeighborList:
    def __init__(self):
        self._list: Dict[int, BelugaNeighbor] = {}
        self._neighbors_update: bool = False
        self._range_update: bool = False
        return

    def update(self, updates: List[Dict[str, Union[int, float]]]):
        for entry in updates:
            if entry["ID"] not in self._list.keys():
                self._list[entry["ID"]] = BelugaNeighbor(entry)
                self._range_update = True
                self._neighbors_update = True
            else:
                self._list[entry["ID"]].update(entry)
                self._range_update = True

    def remove_neighbor(self, neighbor_id: int):
        if neighbor_id in self._list.keys():
            del self._list[neighbor_id]
            self._neighbors_update = True

    def get_updates(self) -> Dict[int, Dict[str, Union[int, float]]]:
        ret = {}
        for x in self._list.values():
            if x.updated:
                ret[x.id] = dict(x)
                x.updated = False
        self._range_update = False
        return ret

    def get_neighbors(self) -> Dict[int, Dict[str, Union[int, float]]]:
        ret = {}
        for x in self._list.values():
            ret[x.id] = dict(x)
        self._neighbors_update = False
        return ret

    def clear(self):
        if self._list:
            self._list.clear()
            self._neighbors_update = True
            self._range_update = False

    @property
    def neighbor_update(self) -> bool:
        return self._neighbors_update

    @property
    def range_update(self) -> bool:
        return self._range_update



class BelugaQueue(mp_queues.Queue):
    def __init__(self, maxsize: int = 1, update_old_items: bool = True):
        self._update = update_old_items
        super().__init__(maxsize=maxsize, ctx=mp.get_context())

    def put(self, obj, block: bool = True, timeout: Optional[float] = None) -> None:
        try:
            super().put(obj, block, timeout)
        except queue.Full:
            try:
                item = self.get_nowait()
            except queue.Empty:
                # Queue item got removed between the put and the get operations. Place new item
                pass
            else:
                if self._update:
                    # This will update the existing keys, add new key-value pairs if not present, and keep keys that
                    # are not present in the new obj unchanged
                    item.update(obj)
                    obj = item
                # else drop the old item...
            super().put(obj, block, timeout)
        return

    def clear(self):
        for _ in itertools.repeat(None, self.qsize()):
            try:
                self.get_nowait()
            except queue.Empty:
                # Queue is now empty
                break


class BelugaSerial:
    # TODO: Discuss how we want to handle case where USB communication fails. Do we want to log the error and try to reconnect? Do we want to terminate the node?
    def __init__(self,
                 baud: int = 115200,
                 timeout: float = 2.0,
                 serial_timeout: float = 0.1,
                 max_lines_read: int = 16,
                 port: Optional[str] = None,
                 neighbor_update_func: Optional[Callable[[dict], None]] = None,
                 range_update_func: Optional[Callable[[dict], None]] = None,
                 range_event: Optional[Callable[[dict], None]] = None,
                 logger_func: Optional[Callable[[Any], None]] = None):

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
        self._neighbors = BelugaNeighborList()

        self._rx_task: Optional[mp.Process] = None
        self._batch_queue: BelugaQueue = BelugaQueue(10, False)

        self._processing_task: Optional[mp.Process] = None
        self._response_q: BelugaQueue = BelugaQueue(update_old_items=False)
        self._command_sent: mp.Event = mp.Event()
        # Used to block the reboot command until done rebooting
        self._reboot_done: mp.Event = mp.Event()

        if neighbor_update_func is None:
            self._neighbors_queue: Optional[BelugaQueue] = BelugaQueue()
            self._neighbors_callback = None
        else:
            self._neighbors_queue = None
            self._neighbors_callback = neighbor_update_func

        if range_update_func is None:
            self._ranges_queue: Optional[BelugaQueue] = BelugaQueue()
            self._ranges_update_callback = None
        else:
            self._ranges_queue = None
            self._ranges_update_callback = range_update_func

        if range_event is None:
            self._range_event_queue: Optional[BelugaQueue] = BelugaQueue()
            self._range_event_callback = None
        else:
            self._range_event_queue = None
            self._range_event_callback = range_event


    def _log(self, s):
        if self._logger is not None:
            self._logger(s)

    def _publish_neighbor_update(self):
        if self._neighbors.neighbor_update:
            if self._neighbors_queue is not None:
                self._neighbors_queue.put(self._neighbors.get_neighbors(), block=False)
            else:
                self._neighbors_callback(self._neighbors.get_neighbors())

    def _publish_range_update(self):
        if self._neighbors.range_update:
            if self._ranges_queue is not None:
                self._ranges_queue.put(self._neighbors.get_updates(), block=False)
            else:
                self._ranges_update_callback(self._neighbors.get_updates())

    def _publish_range_event(self, event):
        if self._range_event_queue is not None:
            self._range_event_queue.put(event, block=False)
        else:
            self._range_event_callback(event)

    def _publish_response(self, response):
        if self._command_sent.is_set():
            self._command_sent.clear()
            self._response_q.put(response)

    def _process_reboot(self, payload):
        if self._ranges_queue is not None:
            self._ranges_queue.clear()
        if self._neighbors_queue is not None:
            self._neighbors_queue.clear()
        if self._range_event_queue is not None:
            self._range_event_queue.clear()
        self._neighbors.clear()
        if self._reboot_done.is_set():
            # Unexpected reboot
            self._log("Beluga rebooted unexpectedly")
            os.kill(os.getppid(), signal.SIGUSR1)
        else:
            self._reboot_done.set()

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

    def _process_frames(self):
        while True:
            frame: BelugaFrame = self._batch_queue.get()
            match frame.type:
                case FrameType.UPDATES:
                    self._neighbors.update(frame.payload)
                case FrameType.EVENT:
                    self._publish_range_event(frame.payload)
                case FrameType.DROP:
                    self._neighbors.remove_neighbor(frame.payload)
                case FrameType.RESPONSE:
                    self._publish_response(frame.payload)
                case FrameType.START:
                    self._process_reboot(frame.payload)
                case _:
                    self._log("Invalid frame type")
            self._publish_neighbor_update()
            self._publish_range_update()

    def _process_frames_entry(self):
        try:
            self._process_frames()
        except KeyboardInterrupt:
            pass # Shutting down

    def _process_rx_buffer(self, buf: bytearray) -> bytearray:
        frame_start, frame_size, _ = BelugaFrame.frame_present(buf)
        if frame_start < 0:
            return buf

        frame = BelugaFrame.extract_frame(buf, frame_start)
        self._batch_queue.put(frame, block=False)
        return buf[frame_start + frame_size:]

    def _read_serial(self):
        rx = b""
        while True:
            if self._serial.inWaiting() > 0:
                rx += self._serial.read(self._serial.inWaiting())
            rx = self._process_rx_buffer(rx)

    def _read_task_entry(self):
        try:
            self._read_serial()
        except KeyboardInterrupt:
            pass # Shutting down

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
        self._reboot_done.clear()
        ret = self._send_command(b'AT+REBOOT\r\n')
        self._reboot_done.wait()
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
        self._processing_task = mp.Process(target=self._process_frames_entry)
        self._rx_task = mp.Process(target=self._read_task_entry)

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
                time.sleep(0.1)
            self._processing_task.close()
        return

    def close(self):
        if self._serial.isOpen():
            self._serial.close()

    def get_neighbors(self) -> Tuple[bool, Dict[int, Dict[str, Union[int, float]]]]:
        # Needed to indicate if the queue is just empty or if the only neighbor got removed
        if self._neighbors_queue is None:
            raise ValueError("Reporting neighbor updates to a callback function")
        update = True
        ret = {}
        try:
            ret = self._neighbors_queue.get_nowait()
        except queue.Empty:
            update = False
        return update, ret

    def get_ranges(self) -> Dict[int, Dict[str, Union[int, float]]]:
        if self._ranges_queue is None:
            raise ValueError("Reporting range updates to a callback function")
        ret = {}
        try:
            ret = self._ranges_queue.get_nowait()
        except queue.Empty:
            pass
        return ret

    def get_range_event(self) -> Dict[int, Dict[str, int]]:
        if self._range_event_queue is None:
            raise ValueError("Reporting range events to a callback function")
        ret = {}
        try:
            ret = self._range_event_queue.get_nowait()
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
        beluga.close()


if __name__ == '__main__':
    main()

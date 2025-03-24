from io import TextIOWrapper
import sys
import os
import signal
import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional, Union, TextIO, Callable, Any, Tuple
import multiprocessing as mp
import queue
import time
from dataclasses import dataclass
import enum
from .beluga_frame import BelugaFrame, FrameType
from .beluga_neighbor import BelugaNeighborList
from .beluga_queue import BelugaQueue

TARGETS = [
    'CMU Beluga',
    'SEGGER J-Link'
]
OPEN_DELAY = 0.5
USB_STILL_ALIVE: Dict[str, bool] = {
    'CMU Beluga': False,
    'SEGGER J-Link': True,
}


@dataclass(init=True)
class BelugaSerialAttr:
    port: str = ""
    baud: int = 115200
    timeout: float = 2.0
    serial_timeout: float = 0.1
    neighbor_update_cb: Optional[Callable[[dict], None]] = None
    range_update_cb: Optional[Callable[[dict], None]] = None
    range_event_cb: Optional[Callable[[dict], None]] = None
    logger_cb: Optional[Callable[[Any], None]] = None


class BelugaSerial:
    class ReconnectionStates(enum.Enum):
        RECONNECT_FIND = enum.auto()
        RECONNECT_SLEEP = enum.auto()
        RECONNECT_CONNECT = enum.auto()
        RECONNECT_UPDATE_SKIPS = enum.auto()
        RECONNECT_NEXT = enum.auto()
        RECONNECT_GET_ID = enum.auto()
        RECONNECT_CHECK_ID = enum.auto()
        RECONNECT_DONE = enum.auto()

    _logger_cb: Optional[Callable[[Any], None]] = None
    _serial: Optional[serial.Serial] = None
    _timeout: float = 2.0
    _neighbors: BelugaNeighborList = BelugaNeighborList()
    _usb_remains_open: bool = False
    _id: int = 0

    _neighbor_queue: Optional[BelugaQueue] = None
    _neighbor_cb: Optional[Callable[[dict], None]] = None
    _range_queue: Optional[BelugaQueue] = None
    _range_cb: Optional[Callable[[dict], None]] = None
    _range_event_queue: Optional[BelugaQueue] = None
    _range_event_cb: Optional[Callable[[dict], None]] = None

    _tasks_running: bool = False

    _rx_task = None
    _batch_queue: BelugaQueue = BelugaQueue(10)

    _processing_task = None
    _response_queue: BelugaQueue = BelugaQueue()
    _command_sent = mp.Event()
    _reboot_done = mp.Event()

    _serial_lock = mp.RLock()

    def __init__(self, attr: Optional[BelugaSerialAttr] = None):
        if attr is None:
            attr = BelugaSerialAttr()

        self._logger_cb = attr.logger_cb

        if not attr.port:
            self._auto_connect(attr)
        else:
            self._serial = serial.Serial(attr.port, attr.baud, timeout=attr.serial_timeout, exclusive=True)

        self._timeout = attr.timeout
        self._neighbor_cb = attr.neighbor_update_cb
        self._range_cb = attr.range_update_cb
        self._range_event_cb = attr.range_event_cb

    def _auto_connect(self, attr: BelugaSerialAttr):
        available_ports = self._find_ports(TARGETS)
        if not available_ports:
            raise FileNotFoundError("Unable to find a given target")
        for target in TARGETS:
            if target not in available_ports.keys():
                continue
            opened = self._open_port(target, attr, available_ports[target])
            if opened and isinstance(self._serial, serial.Serial):
                if not self._serial.is_open:
                    raise FileNotFoundError("Unable to open target")
                break

    def _open_port(self, target: str, attr: BelugaSerialAttr, ports: List[str]) -> bool:
        ret = False
        for port in ports:
            try:
                self._log(f"Trying to connect to {target}: {port}")
                self._serial = serial.Serial(port, attr.baud, timeout=attr.serial_timeout, exclusive=True)
                time.sleep(OPEN_DELAY)
                self._usb_remains_open = USB_STILL_ALIVE[target]
                ret = True
                break
            except serial.SerialException as e:
                self._log(e)
        return ret

    @staticmethod
    def _find_ports(targets: List[str]) -> Dict[str, List[str]]:
        ports = list(list_ports.comports())
        ret: Dict[str, List[str]] = {}

        for port in ports:
            name = f'{port.manufacturer} {port.product}'
            if name in targets:
                if name in ret.keys():
                    ret[name].append(port.device())
                else:
                    ret[name] = [port.device()]
        return ret

    def _log(self, msg: Any):
        if self._logger_cb is not None:
            self._logger_cb(msg)

    def _publish_neighbor_update(self):
        if self._neighbors.neighbor_update:
            updates = self._neighbors.get_neighbors()
            if self._neighbor_cb is None:
                self._neighbor_queue.put(updates, False)
            else:
                self._neighbor_cb(updates)

    def _publish_range_updates(self):
        if self._neighbors.range_update:
            updates = self._neighbors.get_updates()
            if self._range_cb is None:
                self._range_queue.put(updates, False)
            else:
                self._range_cb(updates)

    def _publish_range_event(self, event):
        if self._range_event_cb is None:
            self._range_event_queue.put(event, False)
        else:
            self._range_event_cb(event)

    def _publish_response(self, response):
        if self._command_sent.is_set():
            self._command_sent.clear()
            self._response_queue.put(response)

    def _process_reboot(self, payload):
        self._range_queue.clear()
        self._neighbor_queue.clear()
        self._range_event_queue.clear()
        self._neighbors.clear()
        if self._reboot_done.is_set():
            self._log("Beluga rebooted unexpectedly")
        else:
            self._reboot_done.set()

    def __process_frames(self):
        # TODO: Tasks running variable
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
            self._publish_range_updates()

    def _process_frames(self):
        # TODO: Tasks running variable
        while True:
            try:
                self.__process_frames()
            except Exception as e:
                self._log(f"Uncaught exception: {e}")
                # TODO: Figure out a way to crash entire program

    def _process_rx_buffer(self, buf) -> bytes:
        while True:
            start, size, _ = BelugaFrame.frame_present(buf)
            if start < 0:
                return buf
            frame = BelugaFrame.extract_frame(buf, start)
            self._batch_queue.put(frame, False)
            buf = buf[start + size:]

    def __read_serial(self):
        rx = b""

        # TODO: Tasks running variable
        while True:
            try:
                self._serial_lock.acquire()
                if self._serial.in_waiting > 0:
                    buf = self._serial.read_all()
                    rx += buf
            finally:
                self._serial_lock.release()
            rx = self._process_rx_buffer(rx)

    def _read_serial(self):
        # TODO: Tasks running variable
        while True:
            try:
                self.__read_serial()
            except serial.SerialException:
                # Probably rebooted
                # TODO: Figure out a way to signal program that node rebooted
                pass

    # noinspection PyTypeChecker
    def _send_command(self, command: bytes) -> str:
        not_open: bool = False
        try:
            self._serial_lock.acquire()
            self._serial.write(command)
        except serial.PortNotOpenError:
            not_open = True
        finally:
            self._serial_lock.release()
            if not_open:
                return 'Response timed out'

        try:
            self._command_sent.set()
            response: str = self._response_queue.get(timeout=self._timeout)
        except queue.Empty:
            response = "Response timed out"
        return response

    def start_uwb(self):
        return self._send_command(b"AT+STARTUWB\r\n")

    def stop_uwb(self):
        return self._send_command(b"AT+STOPUWB\r\n")

    def start_ble(self):
        return self._send_command(b"AT+STARTBLE\r\n")

    def stop_ble(self):
        return self._send_command(b"AT+STOPBLE\r\n")

    def _setting(func):
        def inner(self, value: Optional[Union[int, str]] = None):
            at_command = func.__name__.upper()
            if value is not None:
                command = f"AT+{at_command} {value}\r\n".encode()
            else:
                command = f"AT+{at_command}\r\n".encode()
            response = self._send_command(command)
            func(self, response)
            return response
        return inner

    @_setting
    def id(self, new_id: Optional[Union[int, str]] = None):
        if new_id.startswith("ID:") and new_id.endswith("OK"):
            self._id = self._extract_id(new_id)

    @_setting
    def bootmode(self, mode: Optional[int] = None):
        pass

    @_setting
    def rate(self, rate: Optional[int] = None):
        pass

    @_setting
    def channel(self, channel: Optional[int] = None):
        pass

    def reset(self):
        return self._send_command(b"AT+RESET\r\n")

    @_setting
    def timeout(self, timeout: Optional[int] = None):
        pass

    @_setting
    def txpower(self, power: Optional[str] = None):
        pass

    @_setting
    def streammode(self, mode: Optional[int] = None):
        pass

    @_setting
    def twrmode(self, mode: Optional[int] = None):
        pass

    @_setting
    def ledmode(self, mode: Optional[int] = None):
        pass

    def reboot(self):
        response = ""
        if self._usb_remains_open:
            self._reboot_done.clear()
            response = self._send_command(b"AT+REBOOT\r\n")
            self._reboot_done.wait()
        else:
            self._reboot()
        return response

    @_setting
    def pwramp(self, mode: Optional[int] = None):
        pass

    @_setting
    def antenna(self, antenna: Optional[int] = None):
        pass

    def time(self):
        return self._send_command(b"AT+TIME\r\n")

    def _format(self, mode):
        if mode is not None:
            command = f"AT+FORMAT {mode}\r\n".encode()
        else:
            command = b"AT+FORMAT\r\n"
        return self._send_command(command)

    def deepsleep(self):
        return self._send_command(b"AT+DEEPSLEEP\r\n")

    @_setting
    def datarate(self, rate: Optional[int] = None):
        pass

    @_setting
    def preamble(self, length: Optional[int] = None):
        pass

    @_setting
    def pulserate(self, rate: Optional[int] = None):
        pass

    @_setting
    def phr(self, mode: Optional[int] = None):
        pass

    @_setting
    def pac(self, size: Optional[int] = None):
        pass

    @_setting
    def sfd(self, mode: Optional[int] = None):
        pass

    @_setting
    def panid(self, pan: Optional[int] = None):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def _extract_id(self, response) -> int:
        pass

    def _reboot(self):
        pass


if __name__ == "__main__":
    s = BelugaSerial()
    s.bootmode()

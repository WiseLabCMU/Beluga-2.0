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

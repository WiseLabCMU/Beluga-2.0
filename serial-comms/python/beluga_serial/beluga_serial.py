import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional, Union, TextIO, Callable, Any, Tuple, SupportsIndex
import queue
import time
from dataclasses import dataclass
import enum
from .beluga_frame import BelugaFrame, FrameType
from .beluga_neighbor import BelugaNeighborList
from .beluga_queue import BelugaQueue
from concurrent.futures import ThreadPoolExecutor, Future
from concurrent.futures import TimeoutError as FutureTimeoutError
from threading import Event, RLock
import functools
import inspect
import os

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
    port: Optional[str] = None
    baud: int = 115200
    timeout: float = 2.0
    serial_timeout: float = 0.1
    neighbor_update_cb: Optional[Callable[[dict], None]] = None
    range_update_cb: Optional[Callable[[dict], None]] = None
    range_event_cb: Optional[Callable[[dict], None]] = None
    logger_cb: Optional[Callable[[Any], None]] = None
    auto_connect: bool = True


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

    def __init__(self, attr: Optional[BelugaSerialAttr] = None):
        if attr is None:
            attr = BelugaSerialAttr()

        self._logger_cb = attr.logger_cb
        self._serial: Optional[serial.Serial] = None

        if attr.port is None and attr.auto_connect:
            self._auto_connect(attr)
        else:
            self._serial = serial.Serial(attr.port, attr.baud, timeout=attr.serial_timeout, exclusive=True)

        self._timeout = attr.timeout
        self._neighbor_cb = attr.neighbor_update_cb
        self._range_cb = attr.range_update_cb
        self._range_event_cb = attr.range_event_cb

        self._neighbors = BelugaNeighborList()
        self._usb_remains_open = False
        self._id = 0

        self._neighbor_q: Optional[BelugaQueue] = None
        if self._neighbor_cb is None:
            self._neighbor_q = BelugaQueue()
        self._range_q: Optional[BelugaQueue] = None
        if self._range_cb is None:
            self._range_q = BelugaQueue()
        self._range_event_q: Optional[BelugaQueue] = None
        if self._range_event_cb is None:
            self._range_event_q = BelugaQueue()

        self._task_running: bool = False

        self._rx_task: Optional[Future[None]] = None
        self._batch_queue: BelugaQueue = BelugaQueue(10, False)

        self._processing_task: Optional[Future[None]] = None
        self._response_q: BelugaQueue = BelugaQueue(update_old_items=False)

        self._command_sent: Event = Event()
        self._reboot_done: Event = Event()

        self._serial_lock: RLock = RLock()

        self._tasks: ThreadPoolExecutor = ThreadPoolExecutor(max_workers=3)

        self._time_resync: Optional[Callable[[None], None]] = None

    def __del__(self):
        self.stop()
        self._tasks.shutdown()
        self._serial.close()

    def _auto_connect(self, attr: BelugaSerialAttr):
        available_ports = self._find_ports(TARGETS)
        if not available_ports:
            raise FileNotFoundError("Unable to find a given target")
        for target in TARGETS:
            if target not in available_ports.keys():
                continue
            opened = self.__open_port(target, attr, available_ports[target])
            if opened and isinstance(self._serial, serial.Serial):
                if not self._serial.is_open:
                    raise FileNotFoundError("Unable to open target")
                break

    def __open_port(self, target: str, attr: BelugaSerialAttr, ports: List[str]) -> bool:
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
                    ret[name].append(port.device)
                else:
                    ret[name] = [port.device]
        return ret

    def _log(self, msg: Any):
        if self._logger_cb is not None:
            self._logger_cb(msg)

    def _publish_neighbor_update(self):
        if self._neighbors.neighbor_update:
            updates = self._neighbors.get_neighbors()
            if self._neighbor_cb is None:
                self._neighbor_q.put(updates, False)
            else:
                self._neighbor_cb(updates)

    def _publish_range_updates(self):
        if self._neighbors.range_update:
            updates = self._neighbors.get_updates()
            if self._range_cb is None:
                self._range_q.put(updates, False)
            else:
                self._range_cb(updates)

    def _publish_range_event(self, event):
        if self._range_event_cb is None:
            self._range_event_q.put(event, False)
        else:
            self._range_event_cb(event)

    def _publish_response(self, response):
        if self._command_sent.is_set():
            self._response_q.put(response)
            self._command_sent.clear()

    def _process_reboot(self, payload):
        if self._range_q is not None:
            self._range_q.clear()
        if self._neighbor_q is not None:
            self._neighbor_q.clear()
        if self._range_event_q is not None:
            self._range_event_q.clear()
        if self._reboot_done.is_set():
            self._log("Beluga rebooted unexpectedly")
            if self._time_resync is not None:
                self._tasks.submit(self._time_resync)
        else:
            self._reboot_done.set()

    def __process_frames(self):
        while self._task_running:
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
        while self._task_running:
            try:
                self.__process_frames()
            except Exception as e:
                self._log(f"Uncaught exception: {e}")
                os.abort()

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
        while self._task_running:
            if self._serial.in_waiting > 0:
                with self._serial_lock:
                    buf = self._serial.read_all()
                rx += buf
            rx = self._process_rx_buffer(rx)

    def _read_serial(self):
        while self._task_running:
            try:
                self.__read_serial()
            except OSError:
                self._serial.close()
                # Probably rebooted
                time.sleep(OPEN_DELAY)
                try:
                    self._log("Reconnect called from read serial")
                    self._reconnect()
                    if self._time_resync is not None:
                        self._tasks.submit(self._time_resync)
                except RuntimeError as e:
                    self._log(str(e))
                    os.abort()
            except Exception as e:
                self._log(f"An uncaught exception occurred in reading thread: {e}")
                os.abort()

    # noinspection PyTypeChecker
    def _send_command(self, cmd: Optional[str] = None, value: Optional[Union[int, str]] = None) -> str:
        if cmd is None:
            if value is not None:
                command = f"AT+{inspect.stack()[1][3].upper()} {value}\r\n".encode()
            else:
                command = f"AT+{inspect.stack()[1][3].upper()}\r\n".encode()
        else:
            if value is not None:
                command = f"AT+{cmd.upper()} {value}\r\n".encode()
            else:
                command = f"AT+{cmd.upper()}\r\n".encode()

        with self._serial_lock:
            try:
                self._serial.write(command)
            except serial.PortNotOpenError:
                return 'Response timed out'

        try:
            self._command_sent.set()
            response: str = self._response_q.get(timeout=self._timeout)
        except queue.Empty:
            response = "Response timed out"
        return response

    def start_uwb(self):
        return self._send_command("startuwb")

    def stop_uwb(self):
        return self._send_command("stopuwb")

    def start_ble(self):
        return self._send_command("startble")

    def stop_ble(self):
        return self._send_command("stopble")

    def id(self, new_id: Optional[Union[int, str]] = None):
        response = self._send_command(value=new_id)
        if new_id is not None and response.endswith("OK"):
            self._id = self._extract_id(new_id)
        return response

    def bootmode(self, mode: Optional[int] = None):
        return self._send_command(value=mode)

    def rate(self, rate: Optional[int] = None):
        return self._send_command(value=rate)

    def channel(self, channel: Optional[int] = None):
        return self._send_command(value=channel)

    def reset(self):
        return self._send_command()

    def timeout(self, timeout: Optional[int] = None):
        return self._send_command(value=timeout)

    def txpower(self, power: Optional[str] = None):
        return self._send_command(value=power)

    def streammode(self, mode: Optional[int] = None):
        return self._send_command(value=mode)

    def twrmode(self, mode: Optional[int] = None):
        return self._send_command(value=mode)

    def ledmode(self, mode: Optional[int] = None):
        return self._send_command(value=mode)

    def _reboot(self):
        self.stop()
        self._serial.write(b"AT+REBOOT\r\n")
        self._serial.flush()
        self._serial.close()
        self._log("Called reconnect from reboot")
        self._reconnect()
        self.start()

    def reboot(self):
        response = ""
        if self._usb_remains_open:
            self._reboot_done.clear()
            response = self._send_command()
            self._reboot_done.wait()
        else:
            self._reboot()
        return response

    def pwramp(self, mode: Optional[int] = None):
        return self._send_command(value=mode)

    def antenna(self, antenna: Optional[int] = None):
        return self._send_command(value=antenna)

    def time(self):
        return self._send_command()

    def _format(self, mode):
        return self._send_command("format", mode)

    def deepsleep(self):
        return self._send_command()

    def datarate(self, rate: Optional[int] = None):
        return self._send_command(value=rate)

    def preamble(self, length: Optional[int] = None):
        return self._send_command(value=length)

    def pulserate(self, rate: Optional[int] = None):
        return self._send_command(value=rate)

    def phr(self, mode: Optional[int] = None):
        return self._send_command(value=mode)

    def pac(self, size: Optional[int] = None):
        return self._send_command(value=size)

    def sfd(self, mode: Optional[int] = None):
        return self._send_command(value=mode)

    def panid(self, pan: Optional[int] = None):
        return self._send_command(value=pan)

    def start(self):
        if self._task_running or not self._serial.is_open:
            raise RuntimeError("Stop before calling start again")

        self._task_running = True

        self._processing_task = self._tasks.submit(self._process_frames)
        self._rx_task = self._tasks.submit(self._read_serial)

        self._format("2")
        id_ = self.id()
        self._id = self._extract_id(id_)

    def stop(self):
        max_retries = 10
        if not self._task_running:
            return

        self._task_running = False

        while True:
            try:
                self._rx_task.result(timeout=0.01)
            except FutureTimeoutError:
                pass
            else:
                break

        retries = 0
        while retries < max_retries:
            attempts = 0
            while attempts < max_retries:
                try:
                    self._processing_task.result(timeout=0.01)
                except FutureTimeoutError:
                    attempts += 1
                else:
                    break
            if attempts >= max_retries:
                frame = BelugaFrame(FrameType.NO_TYPE, "")
                self._batch_queue.put(frame)
                retries += 1
            else:
                break
        if retries >= max_retries:
            raise RuntimeError("The processing task task is still hanging...")

    def get_neighbors(self) -> Tuple[bool, Dict[int, Dict[str, Union[int, float]]]]:
        ret = True
        list_ = {}
        try:
            list_ = self._neighbor_q.get(False)
        except queue.Empty:
            ret = False
        return ret, list_

    def get_ranges(self) -> Dict[int, Dict[str, Union[int, float]]]:
        list_ = {}
        try:
            list_ = self._range_q.get(False)
        except queue.Empty:
            pass
        return list_

    def get_range_event(self) -> Dict[int, Dict[str, int]]:
        ret = {}
        try:
            ret = self._range_event_q.get(False)
        except queue.Empty:
            pass
        return ret

    @staticmethod
    def _extract_id(response: str) -> int:
        digit = "".join([x for x in response if x.isdigit()])
        return int(digit)

    def _find_port_candidates(self, skip: List[str]) -> List[str]:
        available_ports = self._find_ports(TARGETS)
        candidates = []

        for target in TARGETS:
            if target not in available_ports.keys():
                continue
            for port in available_ports[target]:
                if port in skip:
                    continue
                candidates.append(port)
        return candidates

    def _open_port(self, port) -> bool:
        ret = True
        try:
            self._log(f"Trying to connect to {port}")
            self._serial.close()
            self._serial.port = port
            self._serial.open()
        except serial.SerialException as e:
            self._serial.close()
            self._log(e)
            ret = False
        return ret

    def _get_id_from_device(self) -> str:
        command = b"AT+ID\r\n"
        time.sleep(OPEN_DELAY)
        try:
            self._serial.write(command)
        except serial.SerialException as e:
            if str(e).endswith("Input/output error"):
                return ""
            raise
        time.sleep(OPEN_DELAY)
        response = self._serial.read_all()

        while True:
            start, size, _ = BelugaFrame.frame_present(response)
            if start < 0:
                self._serial.close()
                return ""

            frame = BelugaFrame.extract_frame(response, start)
            if frame.type == FrameType.RESPONSE:
                if frame.payload.startswith("ID:"):
                    return frame.payload
            response = response[start + size:]

    def __reconnect(self):
        state = self.ReconnectionStates.RECONNECT_FIND
        skip = []
        index: SupportsIndex = 0
        port: str = ""
        id_resp: str = ""
        ports = []

        while state != self.ReconnectionStates.RECONNECT_DONE:
            try:
                match state:
                    case self.ReconnectionStates.RECONNECT_FIND:
                        ports = self._find_port_candidates(skip)
                        index = 0
                        # noinspection PyTypeChecker
                        port = ports[index]
                        state = self.ReconnectionStates.RECONNECT_SLEEP if not ports else self.ReconnectionStates.RECONNECT_CONNECT
                    case self.ReconnectionStates.RECONNECT_CONNECT:
                        opened = self._open_port(port)
                        state = self.ReconnectionStates.RECONNECT_GET_ID if opened else self.ReconnectionStates.RECONNECT_NEXT
                    case self.ReconnectionStates.RECONNECT_GET_ID:
                        id_resp = self._get_id_from_device()
                        state = self.ReconnectionStates.RECONNECT_NEXT if not id_resp else self.ReconnectionStates.RECONNECT_CHECK_ID
                    case self.ReconnectionStates.RECONNECT_CHECK_ID:
                        id_ = self._extract_id(id_resp)
                        state = self.ReconnectionStates.RECONNECT_DONE if self._id == id_ else self.ReconnectionStates.RECONNECT_UPDATE_SKIPS
                    case self.ReconnectionStates.RECONNECT_SLEEP:
                        time.sleep(OPEN_DELAY)
                        state = self.ReconnectionStates.RECONNECT_FIND
                    case self.ReconnectionStates.RECONNECT_UPDATE_SKIPS:
                        self._serial.close()
                        skip.append(port)
                        state = self.ReconnectionStates.RECONNECT_NEXT
                    case self.ReconnectionStates.RECONNECT_NEXT:
                        index += 1
                        state = self.ReconnectionStates.RECONNECT_SLEEP if index >= len(
                            ports) else self.ReconnectionStates.RECONNECT_CONNECT
                    case _:
                        self._log("Reached invalid connection state")
                        assert False
            except IndexError:
                # Just reset the state machine
                state = self.ReconnectionStates.RECONNECT_FIND
        self._log(f"Connected to {port}")

    def _reconnect(self):
        with self._serial_lock:
            with ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(functools.partial(self.__reconnect))
                try:
                    future.result(timeout=30.0)
                except FutureTimeoutError:
                    raise RuntimeError("Reconnection timed out")

    def open_target(self, port: str):
        target_ = None
        targets = self.find_ports()
        for target in targets:
            if port in targets[target]:
                target_ = target
                break

        if target_ is None:
            raise FileNotFoundError("Port not found in valid targets")

        self.stop()
        self._serial.close()
        self._usb_remains_open = USB_STILL_ALIVE[target_]
        self._serial.port = port
        self._serial.open()
        self.start()

    def close(self):
        self.stop()
        self._serial.close()

    def find_ports(self) -> Dict[str, List[str]]:
        return self._find_ports(TARGETS)

    def register_resync_cb(self, callback: Optional[Callable[[None], None]]):
        self._time_resync = callback


if __name__ == "__main__":
    from timeit import default_timer as timer

    s = BelugaSerial()
    s.start()

    start = timer()
    s.id()
    end = timer()

    print(end - start)

    print(s.id())
    s.stop()
    print("Done")

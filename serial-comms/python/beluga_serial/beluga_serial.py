#!/user/bin/env python
#
# Abstraction for serial communications with Beluga nodes
#
# Author: Tom Schmitz \<tschmitz@andrew.cmu.edu\>
#
# SPDX-License-Identifier:    BSD-3-Clause

import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict, Optional, Union, Callable, Any, Tuple, SupportsIndex
import queue
import time
from dataclasses import dataclass
import enum
from .beluga_frame import BelugaFrame, FrameType
from .beluga_neighbor import BelugaNeighborList
from .beluga_queue import BelugaQueue
from concurrent.futures import ThreadPoolExecutor, Future
from concurrent.futures import TimeoutError as FutureTimeoutError
from threading import Event, RLock, Lock
import functools
import inspect
import os
import semver

TARGETS = [
    'CMU Beluga',
    'SEGGER J-Link'
]
OPEN_DELAY = 0.5
USB_STILL_ALIVE: Dict[str, bool] = {
    'CMU Beluga': False,
    'SEGGER J-Link': True,
}


class BelugaStatus:
    @dataclass(init=True)
    class BoardInfo:
        id: int = -1
        name: str = ""
        supports_uwb_amp: bool = False
        supports_ble_amp: bool = False
        supports_secondary_ble_antenna: bool = False

    _boards = (
        BoardInfo(0, "Decawave DWM1001-DEV"),
        BoardInfo(1, "CMU Beluga", True, True, True)
    )

    def __init__(self, response):
        board_mask = 0xFF
        ble_mask = 0x1 << 8
        uwb_mask = 0x1 << 9
        antenna_mask = 0x1 << 10
        eviction_mask = 0x1 << 11
        numerical_repr = None

        for x in response.split():
            try:
                numerical_repr = int(x, base=16)
            except ValueError:
                pass

        if numerical_repr is None:
            raise ValueError("Cannot parse Beluga status info")

        board = self._find_board(numerical_repr & board_mask)
        self._name = board.name
        self._uwb_amplifier = board.supports_uwb_amp
        self._ble_amplifier = board.supports_ble_amp
        self._secondary_antenna = board.supports_secondary_ble_antenna
        self._ble_active = (numerical_repr & ble_mask) != 0
        self._uwb_active = (numerical_repr & uwb_mask) != 0
        self._using_second_antenna = (numerical_repr & antenna_mask) != 0
        self._dynamic_eviction_scheme_support = (numerical_repr & eviction_mask) != 0

    def _find_board(self, id_: int) -> BoardInfo:
        for board in self._boards:
            if board.id == id_:
                return board
        raise ValueError("Given board ID is not supported")

    @property
    def name(self):
        return self._name

    @property
    def external_uwb_amp(self):
        return self._uwb_amplifier

    @property
    def external_ble_amp(self):
        return self._ble_amplifier

    @property
    def secondary_antenna_support(self):
        return self._secondary_antenna

    @property
    def ble(self):
        return self._ble_active

    @property
    def uwb(self):
        return self._uwb_active

    @property
    def secondary_antenna(self):
        return self._using_second_antenna

    @property
    def dynamic_eviction_scheme_support(self):
        return self._dynamic_eviction_scheme_support


@dataclass(init=True)
class BelugaSerialAttr:
    """
    Attributes for configuring BelugaSerial

    Attributes:
        port (Optional[str]): The serial port to connect to. Defaults to None.
        baud (int): The baud rate for serial communication. Defaults to 115200.
        timeout (float): The general timeout value in seconds for operations. Defaults to 2.0.
        serial_timeout (float): The timeout for serial communication specifically, in seconds. Defaults to 0.1.
        neighbor_update_cb (Optional[Callable[[dict], None]]): A callback function that gets called when a neighbor update occurs. Defaults to None.
        range_update_cb (Optional[Callable[[dict], None]]): A callback function that gets called when a range update occurs. Defaults to None.
        range_event_cb (Optional[Callable[[dict], None]]): A callback function that gets called when a range event occurs. Defaults to None.
        logger_cb (Optional[Callable[[Any], None]]): A callback function for logging messages. Defaults to None.
        auto_connect (bool): A flag indicating whether to automatically connect to the serial port. Defaults to True.
    """
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
    """
    A class for managing serial communication with Beluga nodes.

    This class handles serial communication, device discovery, command sending,
    and data processing for Beluga nodes. It supports automatic connection,
    command management, and event handling through callbacks or queues.

    The class manages multiple tasks for receiving and processing data frames,
    handling neighbor updates, range updates, and range events.

    Args:
        attr (Optional[BelugaSerialAttr]): Configuration attributes for the serial connection.
            If None, default attributes are used.

    Raises:
        FileNotFoundError: If no valid target device is found during auto-connect
        RuntimeError: If tasks fail to start or stop properly

    Example:
        ```python
        # Create a BelugaSerial instance with default settings
        beluga = BelugaSerial()

        # Start communication
        beluga.start()

        # Send commands
        beluga.id()
        beluga.channel(5)

        # Stop communication
        beluga.stop()
        ```
    """

    class _ReconnectionStates(enum.Enum):
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
        self._clear_neighbors: Event = Event()

        self._serial_lock: RLock = RLock()

        self._tasks: ThreadPoolExecutor = ThreadPoolExecutor(max_workers=3)

        self._time_resync: Optional[Callable[[None], None]] = None

        self._io_hook_lock: Lock = Lock()
        self._io_hooks = []

        self._dropped_uwb_transaction_hook: Optional[Callable[[dict[str, int]], None]] = None

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

    def _record_serial_communication(self, record):
        with self._io_hook_lock:
            for hook in self._io_hooks:
                hook(f"{record}\n")

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

            if self._clear_neighbors.is_set():
                self._neighbors.clear()
                self._clear_neighbors.clear()

            match frame.type:
                case FrameType.UPDATES:
                    self._neighbors.update(frame.payload)
                    self._record_serial_communication(frame.payload)
                case FrameType.EVENT:
                    self._publish_range_event(frame.payload)
                    self._record_serial_communication(frame.payload)
                case FrameType.DROP:
                    self._neighbors.remove_neighbor(frame.payload)
                case FrameType.RESPONSE:
                    self._publish_response(frame.payload)
                case FrameType.START:
                    self._process_reboot(frame.payload)
                case FrameType.RANGING_DROP:
                    if self._dropped_uwb_transaction_hook is not None:
                        self._dropped_uwb_transaction_hook(frame.payload)
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

    def send_raw_input(self, data: str) -> str:
        """
        Sends the given data directly to the Beluga node
        :param data: The data to send to the Beluga node
        :type data: str
        :return: The response from the Beluga node
        :rtype str
        """
        self._record_serial_communication(f"> {data}")
        with self._serial_lock:
            try:
                self._serial.write(f"{data}\r\n".encode())
            except serial.PortNotOpenError:
                self._record_serial_communication("Response timed out")
                return 'Response timed out'

        try:
            self._command_sent.set()
            response: str = self._response_q.get(timeout=self._timeout)
        except queue.Empty:
            response = 'Response timed out'
        self._record_serial_communication(response)
        return response

    # noinspection PyTypeChecker
    def _send_command(self, cmd: Optional[str] = None, value: Optional[Union[int, str]] = None) -> str:
        if cmd is None:
            if value is not None:
                command = f"AT+{inspect.stack()[1][3].upper()} {value}"
            else:
                command = f"AT+{inspect.stack()[1][3].upper()}"
        else:
            if value is not None:
                command = f"AT+{cmd.upper()} {value}"
            else:
                command = f"AT+{cmd.upper()}"

        return self.send_raw_input(command)

    def start_uwb(self):
        """
        Starts the UWB (Ultra-Wideband) radio on the Beluga node.

        Returns:
            str: The command response or 'Response timed out' if the node did not respond
        """
        return self._send_command("startuwb")

    def stop_uwb(self):
        """
        Stops the UWB (Ultra-Wideband) radio on the Beluga node.

        Returns:
            str: The command response or 'Response timed out' if the node did not respond
        """
        return self._send_command("stopuwb")

    def start_ble(self):
        """
        Starts the BLE (Bluetooth Low Energy) radio on the Beluga node.

        Returns:
            str: The command response or 'Response timed out' if the node did not respond
        """
        return self._send_command("startble")

    def stop_ble(self):
        """
        Stops the BLE (Bluetooth Low Energy) radio on the Beluga node.

        Returns:
            str: The command response or 'Response timed out' if the node did not respond
        """
        return self._send_command("stopble")

    def id(self, new_id: Optional[Union[int, str]] = None):
        """
        Sets or gets the boot mode of the Beluga node.

        :param new_id: The boot mode to set:
            * None: Retrieve the current ID (Default)
            * Any value: The new node ID
        :type new_id: Optional[Union[int, str]]
        :return: The command response or a timeout message
        :rtype: str
        """
        response = self._send_command(value=new_id)
        if new_id is not None and response.endswith("OK"):
            self._id = self._extract_id(new_id)
        return response

    def bootmode(self, mode: Optional[int] = None):
        """
        Sets or gets the boot mode of the Beluga node.

        :param mode: The boot mode to set:
            * None: Retrieve the current mode (Default)
            * 0: Both radios off at boot (Reset Value)
            * 1: BLE turned on at boot
            * 2: Both radios turned on at boot
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=mode)

    def rate(self, rate: Optional[int] = None):
        """
        Sets or gets the UWB polling rate of the Beluga node.

        :param rate: The UWB polling rate to set:
            * None: Retrieve the current rate (Default)
            * Any value: The new UWB polling rate
        :type rate: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=rate)

    def channel(self, channel: Optional[int] = None):
        """
        Sets or gets the UWB channel of the Beluga node.

        :param channel: The UWB channel to set:
            * None: Retrieve the current channel (Default)
            * Any value: The new UWB channel
        :type channel: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=channel)

    def reset(self):
        """
        Reset the node settings to their defaults

        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command()

    def timeout(self, timeout: Optional[int] = None):
        """
        Sets or gets the neighbor timeout (ms) of the Beluga node.

        :param timeout: The timeout to set:
            * None: Retrieve the current timeout (Default)
            * Any value: The new timeout
        :type timeout: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=timeout)

    def txpower(self, power: Optional[str] = None):
        """
        Sets or gets the UWB transmission power of the Beluga node.

        :param power: The UWB TX power to set:
            * None: Retrieve the current power (Default)
            * Any value: The new UWB power
        :type power: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=power)

    def streammode(self, mode: Optional[int] = None):
        """
        Sets or gets the stream mode of the Beluga node.

        :param mode: The stream mode to set:
            * None: Retrieve the current channel (Default)
            * 0: Stream mode turned off
            * 1: Stream mode turned on
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=mode)

    def twrmode(self, mode: Optional[int] = None):
        """
        Sets or gets the Two-Way Ranging mode of the Beluga node.

        :param mode: The UWB ranging to set:
            * None: Retrieve the current mode (Default)
            * 0: Single-sided ranging
            * 1: Double-sided ranging
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=mode)

    def ledmode(self, mode: Optional[int] = None):
        """
        Sets or gets the LED mode of the Beluga node.

        :param mode: The LED mode to set:
            * None: Retrieve the current mode (Default)
            * 0: LEDs on
            * 1: LEDs off
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
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
        """
        Reboots the Beluga node
        :return: The command response (or empty string depending on the device) or a timeout message
        :rtype: str
        """
        response = ""
        if self._usb_remains_open:
            self._reboot_done.clear()
            response = self._send_command()
            self._reboot_done.wait()
        else:
            self._reboot()
        if self._time_resync is not None:
            self._tasks.submit(self._time_resync)
        return response

    def pwramp(self, mode: Optional[int] = None):
        """
        Sets or gets the radio power mode of the Beluga node.

        :param mode: The power mode to set:
            * None: Retrieve the current mode (Default)
            * 0: Both external amplifiers OFF
            * 1: BLE Amp OFF, UWB Amp ON
            * 2: BLE Amp LOW, UWB Amp OFF
            * 3: BLE Amp LOW, UWB Amp ON
            * 4: BLE Amp HIGH, UWB Amp OFF
            * 5: BLE Amp HIGH, UWB Amp ON
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=mode)

    def antenna(self, antenna: Optional[int] = None):
        """
        Sets or gets the BLE antenna of the Beluga node.

        :param antenna: The BLE antenna to select:
            * None: Retrieve the current antenna (Default)
            * Any value: The new antenna
        :type antenna: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=antenna)

    def time(self):
        """
        Retrieve the current Beluga node timestamp
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command()

    def _format(self, mode):
        return self._send_command("format", mode)

    def deepsleep(self):
        """
        Place the Beluga node to sleep
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command()

    def datarate(self, rate: Optional[int] = None):
        """
        Sets or gets the UWB data rate of the Beluga node.

        :param rate: The UWB data rate to set:
            * None: Retrieve the current data rate (Default)
            * Any value: The new UWB data rate
        :type rate: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=rate)

    def preamble(self, length: Optional[int] = None):
        """
        Sets or gets the UWB preamble length of the Beluga node.

        :param length: The UWB preamble length to set:
            * None: Retrieve the current length (Default)
            * Any value: The new UWB preamble length
        :type length: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=length)

    def pulserate(self, rate: Optional[int] = None):
        """
        Sets or gets the UWB pulse rate of the Beluga node.

        :param rate: The UWB pulse rate to set:
            * None: Retrieve the current pulse rate (Default)
            * Any value: The new UWB pulse rate
        :type rate: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=rate)

    def phr(self, mode: Optional[int] = None):
        """
        Sets or gets the UWB PHR of the Beluga node.

        :param mode: The UWB PHR to set:
            * None: Retrieve the current PHR (Default)
            * Any value: The new UWB PHR
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=mode)

    def pac(self, size: Optional[int] = None):
        """
        Sets or gets the UWB PAC size of the Beluga node.

        :param size: The UWB PAC size to set:
            * None: Retrieve the current PAC size (Default)
            * Any value: The new UWB PAC size
        :type size: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=size)

    def sfd(self, mode: Optional[int] = None):
        """
        Sets or gets the UWB SFD of the Beluga node.

        :param mode: The UWB SFD to set:
            * None: Retrieve the current SFD (Default)
            * 0: The standard SFD
            * 1: Decawave's proprietary SFD
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=mode)

    def panid(self, pan: Optional[int] = None):
        """
        Sets or gets the UWB PAN ID of the Beluga node.

        :param pan: The UWB PAN ID to set:
            * None: Retrieve the current PAN ID (Default)
            * Any value: The new UWB PAN ID
        :type pan: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=pan)

    def evict(self, scheme: Optional[int] = None):
        """
        Sets or gets the eviction scheme of the Beluga node.

        :param scheme: The eviction scheme to set:
            * None: Retrieve the current scheme (Default)
            * 0: Indexed Round-Robin
            * 1: Lowest RSSI
            * 2: Furthest away
            * 3: Least recently scanned neighbor
            * 4: Least recently ranged to neighbor
        :type scheme: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command(value=scheme)

    def verbose(self, mode: Optional[int] = None):
        """
        Sets or gets the verbose mode of the Beluga node.

        :param mode: The verbose mode to set:
            * None: Retrieve the current verbose mode (Default)
            * 0: verbose mode turned off
            * 1: verbose mode turned on
        :type mode: Optional[int]
        :return: The command response or a timeout message
        :rtype: str
        """

    def status(self):
        """
        Retrieve the current status of the Beluga node
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command()

    def version(self):
        """
        Retrieve the current version of the Beluga node
        :return: The command response or a timeout message
        :rtype: str
        """
        return self._send_command()

    def start(self):
        """
        Starts the processing and receive tasks, sets up the node to transmit
        in the proper mode, and ties the node ID to the instance of BelugaSerial.

        :raises RuntimeError: if tasks are already running or if there is no serial connection to a Beluga node
        :return: None
        """
        if self._task_running or not self._serial.is_open:
            raise RuntimeError("Stop before calling start again")

        self._task_running = True

        self._processing_task = self._tasks.submit(self._process_frames)
        self._rx_task = self._tasks.submit(self._read_serial)

        self._format("2")
        id_ = self.id()
        self._id = self._extract_id(id_)

    def stop(self):
        """
        Stops the processing and receive tasks
        :raises RuntimeError: if it is unable to stop the processing task
        :return: None
        """
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
        self._neighbors.clear()

    def get_neighbors(self) -> Tuple[bool, Dict[int, Dict[str, Union[int, float]]]]:
        """
        Retrieves the current neighbor list.

        :return: True and a dictionary of neighbors if there was a neighbor list update. False and an empty dictionary
        otherwise.
        :rtype: Tuple[bool, Dict[int, Dict[str, Union[int, float]]]]

        :raises AttributeError: if neighbor updates cannot be received through this function
        """

        if self._neighbor_q is None:
            raise AttributeError(
                "Cannot get neighbor updates from `get_neighbors()`. Updates are submitted to the callback function")

        ret = True
        list_ = {}
        try:
            list_ = self._neighbor_q.get(False)
        except queue.Empty:
            ret = False
        return ret, list_

    def get_ranges(self) -> Dict[int, Dict[str, Optional[Union[int, float, dict[str, int]]]]]:
        """
        Retrieves the updated ranges of the list.

        :return: The range updates of the neighbors
        :rtype: Dict[int, Dict[str, Union[int, float]]]

        :raises AttributeError: if range updates cannot be received through this function
        """

        if self._range_q is None:
            raise AttributeError(
                "Cannot get range updates from `get_ranges()`. Updates are submitted to the callback function")

        list_ = {}
        try:
            list_ = self._range_q.get(False)
        except queue.Empty:
            pass
        return list_

    def get_range_event(self) -> Dict[int, Dict[str, int]]:
        """
        Retrieve range exchange events.

        :return: The range events
        :rtype: Dict[int, Dict[str, int]]

        :raises AttributeError: if range events cannot be received through this function
        """

        if self._range_event_q is None:
            raise AttributeError(
                "Cannot get range events from `get_range_event()`. Events are submitted to the callback function")

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
        state = self._ReconnectionStates.RECONNECT_FIND
        skip = []
        index: SupportsIndex = 0
        port: str = ""
        id_resp: str = ""
        ports = []

        while state != self._ReconnectionStates.RECONNECT_DONE:
            try:
                match state:
                    case self._ReconnectionStates.RECONNECT_FIND:
                        ports = self._find_port_candidates(skip)
                        index = 0
                        # noinspection PyTypeChecker
                        port = ports[index]
                        state = self._ReconnectionStates.RECONNECT_SLEEP if not ports else self._ReconnectionStates.RECONNECT_CONNECT
                    case self._ReconnectionStates.RECONNECT_CONNECT:
                        opened = self._open_port(port)
                        state = self._ReconnectionStates.RECONNECT_GET_ID if opened else self._ReconnectionStates.RECONNECT_NEXT
                    case self._ReconnectionStates.RECONNECT_GET_ID:
                        id_resp = self._get_id_from_device()
                        state = self._ReconnectionStates.RECONNECT_NEXT if not id_resp else self._ReconnectionStates.RECONNECT_CHECK_ID
                    case self._ReconnectionStates.RECONNECT_CHECK_ID:
                        id_ = self._extract_id(id_resp)
                        state = self._ReconnectionStates.RECONNECT_DONE if self._id == id_ else self._ReconnectionStates.RECONNECT_UPDATE_SKIPS
                    case self._ReconnectionStates.RECONNECT_SLEEP:
                        time.sleep(OPEN_DELAY)
                        state = self._ReconnectionStates.RECONNECT_FIND
                    case self._ReconnectionStates.RECONNECT_UPDATE_SKIPS:
                        self._serial.close()
                        skip.append(port)
                        state = self._ReconnectionStates.RECONNECT_NEXT
                    case self._ReconnectionStates.RECONNECT_NEXT:
                        index += 1
                        state = self._ReconnectionStates.RECONNECT_SLEEP if index >= len(
                            ports) else self._ReconnectionStates.RECONNECT_CONNECT
                    case _:
                        self._log("Reached invalid connection state")
                        assert False
            except IndexError:
                # Just reset the state machine
                state = self._ReconnectionStates.RECONNECT_FIND
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
        """
        Opens the given port if the port is linked to a valid target. Additionally,
        automatically starts the tasks if the port is successfully opened.

        :param port: The serial port to open
        :return: None

        :raises FileNotFoundError: Port is not associated with a valid target
        """
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
        """
        Stop the processing and receive tasks and close the serial port
        :return: None
        """
        self.stop()
        self._serial.close()

    def find_ports(self) -> Dict[str, List[str]]:
        """
        Find the ports associated with valid targets
        :return: Valid targets
        :rtype: Dict[str, List[str]]
        """
        return self._find_ports(TARGETS)

    def register_resync_cb(self, callback: Optional[Callable[[None], None]]):
        """
        Registers a time resynchronization callback. This is called whenever the node reboots.

        :param callback: The function entry for time resynchronization. If None, this unregisters the resynchronization method.

        :type callback: Optional[Callable[[None], None]]
        :return: None
        """
        self._time_resync = callback

    def register_io_hook(self, hook: Callable[[str], None]):
        """
        Registers a io callback. The IO callbacks echo the AT commands and their results. Additionally, range updates
        and exchange events are also reported through the IO callbacks.

        :param hook: An IO hook where serial communications are recorded
        :type hook: Callable[[str], None]

        :return: None

        :raises ValueError: If `hook` is not a callable
        """
        if not callable(hook):
            raise ValueError("`hook` must be a callable")
        self._io_hooks.append(hook)

    def register_dropped_uwb_exchange_hook(self, hook: Optional[Callable[[dict[str, int]], None]]):
        """
        Registers or unregisters a hook for reporting dropped UWB exchanges.

        :param hook: The hook to register. If None, this unregisters the current hook
        :type hook: Optional[Callable[[str], None]]

        :return: None

        :raises ValueError: ``If `hook` is not a callable
        """
        if hook is None:
            self._dropped_uwb_transaction_hook = None
        if not callable(hook):
            raise ValueError("`hook` must be a callable")
        self._dropped_uwb_transaction_hook = hook

    def clear(self):
        """
        Clears the neighbor list
        """
        self._clear_neighbors.set()


def unpack_beluga_status(response: str) -> BelugaStatus:
    """
    Parses and unpacks the status message from the Beluga node's AT+STATUS command
    :param response: The response from the AT+STATUS command
    :type response: str
    :return: The unpacked data from the status response
    :rtype: BelugaStatus
    :raises ValueError: If the status could not be parsed from the response
    """
    return BelugaStatus(response)


def unpack_beluga_version(response: str) -> semver.Version:
    """
    Parses the node version from the response string for the AT+VERSION command
    :param response: The response from the AT+VERSION command
    :type response: str
    :return: Sematic version info
    :rtype: semver.Version
    :raises ValueError: If the version string could not be parsed
    """
    return semver.Version.parse(response.split()[0])


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

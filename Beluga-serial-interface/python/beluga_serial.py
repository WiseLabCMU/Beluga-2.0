import serial
import serial.tools.list_ports as list_ports
from typing import List, Dict
import enum


TARGETS = [
    'CMU Beluga'
]


class BelugaBootMode(enum.IntEnum):
    WIRELESS_OFF = 0
    BLE_ON = 1
    BLE_UWB_ON = 2


class BelugaSerial:
    def __init__(self, baud: int = 115200):
        targets = self._find_ports(TARGETS)
        if not targets:
            raise FileNotFoundError(f'Unable to find a given target. Valid targets: {TARGETS}')
        for target in TARGETS:
            if target in targets.keys():
                print(f"Connecting to {target}: {targets[target][0]}")
                self._serial = serial.Serial(targets[target][0], baudrate=baud)
                break

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

    def _send_command(self, command: bytes) -> str:
        pass

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

    def rate(self, rate: int):
        command = f'AT+RATE {rate}\r\n'
        ret = self._send_command(command.encode())
        return ret



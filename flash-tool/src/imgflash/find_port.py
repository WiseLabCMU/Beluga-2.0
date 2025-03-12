import asyncio
from serial.tools import list_ports
from smpclient import SMPClient, success
from smpclient.requests.os_management import EchoWrite
from smpclient.transport.serial import SMPSerialTransport
from typing import List, Dict
from rich.progress import Progress, BarColumn, TextColumn, TaskProgressColumn, TimeElapsedColumn


class PortProgress:
    def __init__(self, ports: Dict[str, List[str]]):
        self._ports = [item for sublist in ports.values() for item in sublist]
        self._targets = {value: key for key, values in ports.items() for value in values}
        self._total = len(self._ports)
        self._progress = Progress(TextColumn("Testing ports..."), BarColumn(), TaskProgressColumn(), TextColumn("[magenta]([progress.description]{task.description})"), TimeElapsedColumn())
        self._task = self._progress.add_task("[cyan]Testing ports...", total=self._total)

    def __iter__(self):
        for index, port in enumerate(self._ports, start=1):
            self._progress.update(self._task, advance=1, description=f"{port} [{index}/{self._total}]")
            yield port, self._targets[port]

    def __enter__(self):
        self._progress.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._progress.stop()


def _find_all_ports() -> Dict[str, List[str]]:
    ports = list_ports.comports()
    found: Dict[str, List[str]] = {}

    for port in ports:
        target = f'{port.manufacturer} {port.product}'
        if target in found.keys():
            found[target].append(port.device)
        else:
            found[target] = [port.device]
    return found


def _find_ports(targets: List[str]) -> Dict[str, List[str]]:
    ports = list_ports.comports()
    found: Dict[str, List[str]] = {}

    if not targets:
        return _find_all_ports()

    for target in targets:
        for port in ports:
            if f'{port.manufacturer} {port.product}' == target:
                if target in found.keys():
                    found[target].append(port.device)
                else:
                    found[target] = [port.device]
    return found


async def _test_port(port: str) -> bool:
    try:
        async with SMPClient(SMPSerialTransport(), port) as client:
            response = await client.request(EchoWrite(d="Hello, World!"), timeout_s=0.5)
            if success(response):
                return True
            else:
                return False
    except TimeoutError:
        return False


def test_port(port: str) -> bool:
    return asyncio.run(_test_port(port))


async def _find_mcumgr_ports_progress(targets: Dict[str, List[str]]) -> Dict[str, List[str]]:
    valid_targets: Dict[str, List[str]] = {}
    with PortProgress(targets) as progress:
        for port, target in progress:
            result = await _test_port(port)
            if result:
                if target in valid_targets.keys():
                    valid_targets[target].append(port)
                else:
                    valid_targets[target] = [port]
    return valid_targets


async def _find_mcumgr_ports_no_progress(targets: Dict[str, List[str]]) -> Dict[str, List[str]]:
    valid_targets: Dict[str, List[str]] = {}
    for target in targets:
        for port in targets[target]:
            result = await _test_port(port)
            if result:
                if target in valid_targets.keys():
                    valid_targets[target].append(port)
                else:
                    valid_targets[target] = [port]
    return valid_targets

async def _find_mcumgr_ports(search_targets: List[str], progress: bool = False) -> Dict[str, List[str]]:
    targets = _find_ports(search_targets)
    if progress:
        return await _find_mcumgr_ports_progress(targets)
    return await _find_mcumgr_ports_no_progress(targets)


def find_mcumgr_ports(targets: List[str], progress: bool = False) -> Dict[str, List[str]]:
    return asyncio.run(_find_mcumgr_ports(targets, progress))


if __name__ == "__main__":
    from smpclient import logger
    import logging
    logger.level = logging.FATAL
    TARGETS = [
        'CMU Beluga'
    ]
    print(find_mcumgr_ports([], True))

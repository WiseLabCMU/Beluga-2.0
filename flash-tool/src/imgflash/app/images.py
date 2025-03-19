from imgflash.find_port import find_mcumgr_ports, test_port
from imgflash.image_manager import read_image_states, ImageState
import typer
from typing_extensions import Annotated
from smpclient import logger
import logging
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from serial.tools import list_ports
from typing import List, Optional
from .utils import print_error


app = typer.Typer()
console = Console()


class PortInfo:
    def __init__(self, target: str, port: str):
        self._target = target
        self._port = port
        self._states: Optional[List[ImageState]] = None

    @property
    def target(self):
        return self._target

    @property
    def port(self):
        return self._port

    @property
    def states(self):
        return self._states

    @states.setter
    def states(self, value: List[ImageState]):
        self._states = value

    def __str__(self):
        return f"{self._target} - {self._port}"

    def __repr__(self):
        return self.__str__()


def _no_port(all_: bool, targets) -> List[PortInfo]:
    found = find_mcumgr_ports(targets, True)

    if not found:
        print_error("No SMP servers found")

    if len(found.keys()) > 1 and not all_:
        print_error("Please specify port with [cyan]--port[/cyan] or [cyan]-p[/cyan] or run with the [cyan]--all[/cyan] flag")
    if len(found.keys()) == 1:
        ports = list(found.values())[0]
        if len(ports) > 1 and not all_:
            print_error("Please specify port with [cyan]--port[/cyan] or [cyan]-p[/cyan] or run with the [cyan]--all[/cyan] flag")

    return [PortInfo(target, port) for target in found for port in found[target]]


def _port(port: str) -> List[PortInfo]:
    ports = list_ports.comports()
    target = [f"{__port.manufacturer} {__port.product}" for __port in ports if __port.device == port]
    if not target:
        print_error("Specified port not found")
    if not test_port(port):
        print_error("SMP server not running on specified port")
    return [PortInfo(target[0], port)]


def _tabulate_single(port: PortInfo) -> Table:
    table = Table("Image Info", "Slot 0", show_lines=True)
    state: ImageState = port.states[0]
    table.add_row("[cyan]Hash", f"[magenta]{state.hash.hex()}")
    table.add_row("[cyan]Version", f"[magenta]{state.version}")
    table.add_row("[cyan]Active", ":white_check_mark:" if state.active else ":x:")
    table.add_row("[cyan]Bootable", ":white_check_mark:" if state.bootable else ":x:")
    table.add_row("[cyan]Confirmed", ":white_check_mark:" if state.confirmed else ":x:")
    table.add_row("[cyan]Pending", ":white_check_mark:" if state.pending else ":x:")
    table.add_row("[cyan]Permanent", ":white_check_mark:" if state.permanent else ":x:")
    return table


def _tabulate_double(port: PortInfo) -> Table:
    table = Table("Image Info", "Slot 0", "Slot 1", show_lines=True)
    state0: ImageState = port.states[0]
    state1: ImageState = port.states[1]
    table.add_row("[cyan]Hash", f"[magenta]{state0.hash.hex()}", f"[green]{state1.hash.hex()}")
    table.add_row("[cyan]Version", f"[magenta]{state0.version}", f"[green]{state1.version}")
    table.add_row("[cyan]Active", ":white_check_mark:" if state0.active else ":x:", ":white_check_mark:" if state1.active else ":x:")
    table.add_row("[cyan]Bootable", ":white_check_mark:" if state0.bootable else ":x:", ":white_check_mark:" if state1.bootable else ":x:")
    table.add_row("[cyan]Confirmed", ":white_check_mark:" if state0.confirmed else ":x:", ":white_check_mark:" if state1.confirmed else ":x:")
    table.add_row("[cyan]Pending", ":white_check_mark:" if state0.pending else ":x:", ":white_check_mark:" if state1.pending else ":x:")
    table.add_row("[cyan]Permanent", ":white_check_mark:" if state0.permanent else ":x:", ":white_check_mark:" if state1.permanent else ":x:")
    return table

def _tabulate(port: PortInfo):
    if len(port.states) > 1:
        table = _tabulate_double(port)
    else:
        table = _tabulate_single(port)
    console.print(Panel(table, title=str(port), title_align="left", border_style="cyan", expand=True))



@app.command("images", help="Gather image states of a connected device")
def images(port: Annotated[str, typer.Option("--port", "-p", help="Port communicates with the SMP server")] = None,
           all_: Annotated[bool, typer.Option("--all", help="Display the image information for all the connected devices")] = False,
           log: Annotated[bool, typer.Option(help="Display logger information")] = False,
           targets: Annotated[str, typer.Option(help='List of targets to search for. Targets are described as "Manufacturer Product". Separate targets with commas (,)')] = None):
    if targets is None:
        _targets = []
    else:
        _targets = targets.split(",")

    if not log:
        logger.level = logging.FATAL

    if port is None:
        ports = _no_port(all_, _targets)
    else:
        ports = _port(port)

    for port in ports:
        states = read_image_states(port.port)
        port.states = states
        _tabulate(port)

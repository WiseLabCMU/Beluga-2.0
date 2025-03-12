import typer
from rich.panel import Panel
from rich.console import Console
from typing import Any
from serial.tools import list_ports
from imgflash.find_port import test_port


console = Console()


def print_error(message: Any):
    console.print(Panel(
        f"[red bold]{message}[/red bold]", title="[red]Error[/red]", title_align="left", style="red", expand=True))
    raise typer.Exit(code=1)


def print_warning(message: Any, die: bool = False):
    console.print(Panel(message, title="[yellow]Warning[/yellow]", title_align="left", expand=True, style="yellow"))
    if die:
        raise typer.Exit(code=1)


def print_success(message: Any):
    console.print(Panel(message, title="[green]Success[/green]", title_align="left", expand=True, style="green"))


def check_valid_port(port: str):
    ports = [_port.device for _port in list_ports.comports()]
    if port not in ports:
        print_error("Given port is not valid")
    if not test_port(port):
        print_error("Given port is not running an SMP server")

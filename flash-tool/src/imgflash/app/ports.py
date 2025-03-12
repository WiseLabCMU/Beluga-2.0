from imgflash.find_port import find_mcumgr_ports
import typer
from typing_extensions import Annotated
from smpclient import logger
import logging
from rich.console import Console
from rich.panel import Panel


app = typer.Typer()
console = Console()


@app.command(name="ports", help="Find all serial ports running an SMP server")
def ports(targets: Annotated[str, typer.Option(help='List of targets to search for. Targets are described as "Manufacturer Product". Separate targets with commas (,)')] = None,
          log: Annotated[bool, typer.Option(help="Allow logger output")] = False):
    if targets is not None:
        search_targets = targets.split(",")
    else:
        if not typer.confirm("This operation may take a while. Are you sure you want to proceed without specifying targets?"):
            return
        search_targets = []

    if not log:
        logger.level = logging.FATAL

    found = find_mcumgr_ports(search_targets, True)

    for target in found:
        message = '\n'.join(found[target])
        console.print(
            Panel(message,
                  title=target,
                  title_align="left",
                  border_style="cyan",
                  expand=True))

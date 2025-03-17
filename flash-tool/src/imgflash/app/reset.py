from imgflash.image_manager import reset_mcu
import typer
from typing_extensions import Annotated
from smpclient import logger
import logging
from .utils import check_valid_port, print_success


app = typer.Typer()


@app.command("reset", help="Reset the connected microcontroller")
def reset(port: Annotated[str, typer.Argument(help="Port that the SMP server is running on")],
          force: Annotated[bool, typer.Option("--force", "-f", help="Force the reset")] = False,
          log: Annotated[bool, typer.Option(help="Show logger output")] = False):
    if not log:
        logger.level = logging.FATAL

    check_valid_port(port)

    reset_mcu(port, force)
    print_success("Reset complete")

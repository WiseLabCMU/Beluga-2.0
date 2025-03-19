from imgflash.image_manager import mark_slot_pending
from .utils import check_valid_port, print_success
from .images import images
import typer
from typing_extensions import Annotated
from smpclient import logger
import logging


app = typer.Typer()


@app.command('swap', help='Mark the secondary image as pending')
def swap(port: Annotated[str, typer.Argument(help="The port the SMP server is running on")],
         display_images: Annotated[bool, typer.Option("--show", "-l", help="Show image slots after confirming")] = False,
         log: Annotated[bool, typer.Option(help="Show logger output")] = False):
    if not log:
        logger.level = logging.FATAL
    check_valid_port(port)
    mark_slot_pending(port, 1)
    if not display_images:
        print_success("Successfully marked the image as pending")
    else:
        images(port)

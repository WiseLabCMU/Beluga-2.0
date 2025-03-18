from imgflash.image_manager import confirm_image
import typer
from typing_extensions import Annotated
from .images import images
from smpclient import logger
import logging
from .utils import check_valid_port, print_error, print_success

app = typer.Typer()


@app.command("confirm", help="Confirm the active image if it has not been confirmed already")
def confirm(port: Annotated[str, typer.Argument(help="Port to confirm the image on")],
            show_images: Annotated[bool, typer.Option("--show", "-l",
                                                      help="Show image slots after confirming")] = False,
            slot: Annotated[int, typer.Option(help="Slot to confirm")] = 0,
            force: Annotated[bool, typer.Option("--force", "-f",
                                                help="Ignore the confirmed and running "
                                                     "image states when confirming")] = False,
            log: Annotated[bool, typer.Option(help="Show logger output")] = False):
    if not log:
        logger.level = logging.FATAL

    check_valid_port(port)

    try:
        confirm_image(port, slot, force)
        print_success("Confirmed the image")
    except Exception as e:
        print_error(e)

    if show_images:
        images(port)

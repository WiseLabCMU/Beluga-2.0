from imgflash.image_manager import upload_image, UploadStatusBase, reset_mcu, ImageManagerException
import typer
from typing_extensions import Annotated
from smpclient import logger
import logging
from rich.console import Console
from pathlib import Path
from .utils import print_error, check_valid_port
from rich.progress import Progress, TextColumn, BarColumn, TaskProgressColumn, TimeElapsedColumn
import time
from typing import Optional


app = typer.Typer()
console = Console()


class UploadBar(UploadStatusBase):
    def __init__(self, image_len: int):
        super().__init__(image_len)
        self._time_s: Optional[float] = None
        self._progress = Progress(TextColumn("Uploading..."),
                                  BarColumn(),
                                  TaskProgressColumn(),
                                  TextColumn("[magenta]([progress.description]{task.description})"),
                                  TimeElapsedColumn())
        self._task = self._progress.add_task("[magenta]0.0 KB/s", total=image_len)

    def __enter__(self):
        self._progress.start()
        self._time_s = time.time()
        return super().__enter__()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            self._progress.live.transient = True
        self._progress.stop()
        if exc_type is not None:
            self._progress.console.print("[red bold]Upload failed[/red bold]")

    def update(self, offset: int) -> None:
        rate = f"{offset / (time.time() - self._time_s) / 1000:.2f} KB/s"
        self._progress.update(self._task, completed=offset, description=rate)


@app.command("flash", help="Upload an image to a device")
def images(
        port: Annotated[str, typer.Argument(help="Port to upload the image on")],
        build_dir: Annotated[Path, typer.Argument(help="The path to the build directory")],
        app_name: Annotated[str, typer.Argument(help="Name of the application to be uploaded")],
        slot: Annotated[int, typer.Option(help="Slot to save the image to.")] = 0,
        reset: Annotated[bool, typer.Option("--reset","-r", help="Reset the microcontroller when finished")] = False,
        log: Annotated[bool, typer.Option(help="Enable the logger")] = False):
    if not log:
        logger.level = logging.FATAL
    check_valid_port(port)

    if not build_dir.is_dir():
        print_error("Given directory does not exist")
    app_dir = build_dir / app_name
    if not app_dir.is_dir():
        print_error("Given app does not exist")
    try:
        upload_image(port, build_dir, app_name, slot, UploadBar)
    except ImageManagerException:
        print_error(f"Unable to flash on {port}. Max retries reached.")

    if reset:
        reset_mcu(port)

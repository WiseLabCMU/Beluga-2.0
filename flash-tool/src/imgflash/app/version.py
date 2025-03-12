import importlib.metadata
import typer


app = typer.Typer()

__version__ = importlib.metadata.version("imgflash")

@app.command(name="version", help="Check the current version of imgflash")
def version():
    print(__version__)

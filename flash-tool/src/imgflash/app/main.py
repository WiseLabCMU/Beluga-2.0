#!/usr/bin/env python3
import importlib
import pkgutil
import typer
import os


class ImgFlash:
    app = typer.Typer()

    def __init__(self):
        for _, mod_name, _ in pkgutil.iter_modules([os.path.dirname(os.path.abspath(__file__))]):
            module = importlib.import_module(f".{mod_name}", package="imgflash.app")
            if hasattr(module, "app"):
                if hasattr(module, "name"):
                    self.app.add_typer(module.app, name=module.name)
                else:
                    self.app.add_typer(module.app)

    def run(self):
        self.app()


def main():
    app = ImgFlash()
    app.run()


if __name__ == "__main__":
    main()
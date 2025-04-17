from PyQt5.QtWidgets import QFileDialog, QWidget
from typing import Optional, Iterable
from dataclasses import dataclass


@dataclass(init=True)
class FileFilter:
    extension: str = ""
    description: str = ""

    def __str__(self):
        return f"{self.description} (*.{self.extension})"


class FileDialog(QFileDialog):
    def __init__(self, parent: Optional[QWidget] = None, filters: Optional[Iterable[FileFilter]] = None):
        super().__init__(parent)
        self.setWindowTitle("Select or create save file")
        if filters is not None:
            self.setNameFilters([str(x) for x in filters])
        self.setFileMode(QFileDialog.FileMode.AnyFile)
        self.setAcceptMode(QFileDialog.AcceptMode.AcceptSave)
        self.setViewMode(QFileDialog.ViewMode.Detail)
        self.filters = filters

    def exec(self):
        if super().exec():
            filename = self.selectedFiles()[0]
            filter_ = None
            for x in self.filters:
                if str(x) == self.selectedNameFilter():
                    filter_ = x
                    break
            assert filter_ is not None
            if not filename.endswith(f".{filter_.extension}"):
                if filename[-1] != ".":
                    filename += "."
                filename += filter_.extension
            return filename
        return None

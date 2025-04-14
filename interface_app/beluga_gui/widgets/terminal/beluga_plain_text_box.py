from PyQt5.QtWidgets import QWidget, QPlainTextEdit, QApplication
from PyQt5.QtCore import QObject, QEvent, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QKeyEvent, QKeySequence
from typing import Optional


class BelugaTerminal(QPlainTextEdit):
    enter_pressed = pyqtSignal()
    _maximum_command_storage = 128

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.installEventFilter(self)
        self._slider_shown = False
        self._dat_out_updated = False
        self._line_edit = False
        self._current_position = 0
        self._item_text: list[str] = []
        self._data_out_str: str = ""
        self._data_out_cursor_pos: int = 0
        self._open = False


    def _handle_vertical_scroll_bar_event(self, event):
        match event.type():
            case QEvent.MouseButtonRelease:
                QTimer.singleShot(1, self.update_display)
            case QEvent.UpdateLater:
                if not self._slider_shown:
                    # Slider has been shown, scroll down to the bottom of the text edit if cursor position is at the end
                    if self.textCursor().atEnd():
                        self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())
                    self._slider_shown = True
            case QEvent.Hide:
                self._slider_shown = False

    def _handle_up_key(self):
        if self._current_position > 0:
            self._current_position -= 1
        self._data_out_str = self._item_text[self._current_position]
        self._data_out_cursor_pos = len(self._data_out_str)
        self.update_display()
        return True

    def _handle_down_key(self):
        if self._current_position < len(self._item_text):
            self._current_position += 1
        self._data_out_str = self._item_text[self._current_position]
        self._data_out_cursor_pos = len(self._data_out_str)
        self.update_display()
        return True

    def _handle_enter_pressed(self):
        if not self._open:
            return True
        second_cond = len(self._item_text) == 0
        if not second_cond:
            second_cond = self._data_out_str != self._item_text[-1]
        if self._data_out_str and second_cond:
            # Previous entry is not the same as this entry
            if len(self._item_text) >= self._maximum_command_storage:
                self._item_text = self._item_text[1:]
            self._item_text.append(self._data_out_str)
        self._current_position = len(self._item_text)
        self.enter_pressed.emit()
        return True

    def _handle_backspace(self, event: QKeyEvent):
        if event.modifiers() & Qt.ControlModifier:
            # Delete entire word
            pass
        elif True: # TODO
            pass
        self.update_display()
        self.update_cursor()
        return True

    def _handle_key_press_line_edit(self, key_event: QKeyEvent):
        if key_event.key() == Qt.Key_Up and not (key_event.modifiers() & Qt.ShiftModifier):
            # Up pressed without holding shift
            return self._handle_up_key()
        if key_event.key() == Qt.Key_Down and not (key_event.modifiers() & Qt.ShiftModifier):
            # Up pressed without holding shift
            return self._handle_down_key()
        if (key_event.key() == Qt.Key_Return or key_event.key() == Qt.Key_Enter) and not (key_event.modifiers() & Qt.ControlModifier) and not (key_event.modifiers() & Qt.ShiftModifier):
            # Enter pressed
            return self._handle_enter_pressed()
        if key_event.key() == Qt.Key_Backspace:
            return self._handle_backspace(key_event)

    def _handle_key_press(self, target, event: QEvent):
        key_event = QKeyEvent(event)
        if (key_event.modifiers() & Qt.ControlModifier) == Qt.ControlModifier:
            # Check if this is a shortcut for cut
            if QKeySequence(key_event.key() | Qt.ControlModifier) == QKeySequence.Cut:
                QApplication.clipboard().setText(self.textCursor().selection().toPlainText())
                return True
        self._dat_out_updated = True

    def eventFilter(self, target: QObject, event: QEvent):
        if target == self.verticalScrollBar():
            self._handle_vertical_scroll_bar_event(event)
        elif event.type() == QEvent.KeyPress:
            ret = self._handle_key_press(target, event)
            if ret is not None:
                return ret

        return QObject().eventFilter(target, event)

    def update_display(self):
        print("Update display")

    def update_cursor(self):
        print("Update cursor")

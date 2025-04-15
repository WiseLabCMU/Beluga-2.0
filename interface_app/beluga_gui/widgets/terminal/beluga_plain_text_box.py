from PyQt5.QtWidgets import QWidget, QPlainTextEdit, QApplication
from PyQt5.QtCore import QObject, QEvent, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QKeyEvent, QKeySequence
from typing import Optional
from collections import deque


class CommandHistory:
    def __init__(self, max_depth: int = 512):
        # List of previous commands
        self._previous_commands = deque(maxlen=max_depth)
        # The current command string
        self._current_str: str = ""
        # Item in the list. If len, then current item is the one being modified
        self._current_item: int = 0
        # Displayed string. This is the one that gets saved when enter is pressed
        self._current_displayed_str: str = ""
        # Cursor position
        self._cursor_position: int = 0

    @property
    def display_str(self):
        return self._current_displayed_str

    @property
    def cursor_position(self):
        return self._cursor_position

    @cursor_position.setter
    def cursor_position(self, pos: int):
        if not isinstance(pos, int):
            raise ValueError("`pos` must be an integer")
        self._cursor_position = pos

    def key_right(self, word: bool):
        if word:
            while self._cursor_position < len(self._current_displayed_str):
                self._cursor_position += 1
                if (self._current_displayed_str[self._cursor_position] == ' ' or
                        self._current_displayed_str[self._cursor_position] == '\r' or
                        self._current_displayed_str[self._cursor_position] == '\n'):
                    break
        elif self._cursor_position < len(self._current_displayed_str):
            self._cursor_position += 1

    def key_left(self, word: bool):

        if word:
            while self._cursor_position > 0:
                self._cursor_position -= 1
                if (self._current_displayed_str[self._cursor_position] == ' ' or
                        self._current_displayed_str[self._cursor_position] == '\r' or
                        self._current_displayed_str[self._cursor_position] == '\n'):
                    break
        elif self._cursor_position > 0:
            self._cursor_position -= 1

    def key_up(self):
        if self._current_item > 0:
            self._current_item -= 1
        self._current_displayed_str = self._previous_commands[self._current_item]
        self._cursor_position = len(self._current_displayed_str)

    def key_down(self):
        if self._current_item < len(self._previous_commands):
            self._current_item += 1
        if self._current_item == len(self._previous_commands):
            self._current_displayed_str = self._current_str
        else:
            self._current_displayed_str = self._previous_commands[self._current_item]
        self._cursor_position = len(self._current_displayed_str)

    def enter_pressed(self):
        second_cond = not self._previous_commands
        if not second_cond:
            second_cond = self._current_displayed_str != self._previous_commands[-1]
        if self._current_displayed_str and second_cond:
            self._previous_commands.append(self._current_displayed_str)
        self._current_str = ""
        self._current_displayed_str = ""
        self._current_item = len(self._previous_commands)
        self._cursor_position = 0

    @property
    def command(self):
        if not self._previous_commands:
            return ""
        return self._previous_commands[-1]

    def _using_history(self) -> bool:
        return not (self._current_displayed_str == self._current_str and self._current_item == len(self._previous_commands))

    def append_char(self, c: str):
        live_text = not self._using_history()

        if self._cursor_position == len(self._current_displayed_str):
            self._current_displayed_str += c
        else:
            self._current_displayed_str = (self._current_displayed_str[:self._cursor_position] + c +
                                           self._current_displayed_str[self._cursor_position + 1:])
        self._cursor_position += 1

        if live_text:
            self._current_str = self._current_displayed_str

    def backspace(self, remove_word: bool):
        live_text = not self._using_history()

        if not self._current_displayed_str.strip():
            return

        if remove_word:
            part1 = self._current_displayed_str[:self._cursor_position]
            part2 = self._current_displayed_str[self._cursor_position:]
            part1 = part1.strip().rsplit(' ', 1)[0]
            self._cursor_position = len(part1) + 1
            if part1.strip():
                part1 += ' '
            self._current_displayed_str = part1 + part2

        else:
            if self._cursor_position > 0:
                self._cursor_position -= 1
            else:
                return
            self._current_displayed_str = self._current_displayed_str[:self._cursor_position] + self._current_displayed_str[self._cursor_position + 1:]

        if live_text:
            self._current_str = self._current_displayed_str



class BelugaTerminal(QPlainTextEdit):
    enter_pressed = pyqtSignal()
    _maximum_command_storage = 128

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.installEventFilter(self)
        self._slider_shown = False
        self._dat_out_updated = False
        self._line_edit = False
        self._history = CommandHistory()
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
        self._history.key_up()
        self.update_display()
        return True

    def _handle_down_key(self):
        self._history.key_down()
        self.update_display()
        return True

    def _handle_enter_pressed(self):
        if not self._open:
            return True
        self._history.enter_pressed()
        self.enter_pressed.emit()
        return True

    def _handle_backspace(self, event: QKeyEvent):
        if event.modifiers() & Qt.ControlModifier:
            self._history.backspace(True)
        else:
            self._history.backspace(False)
        self.update_display()
        self.update_cursor()
        return True

    def _handle_left_key(self, event: QKeyEvent):
        if event.modifiers() & Qt.ControlModifier:
            self._history.key_left(True)
        else:
            self._history.key_left(False)
        self.update_cursor()
        return True

    def _handle_right_key(self, event: QKeyEvent):
        if event.modifiers() & Qt.ControlModifier:
            self._history.key_right(True)
        else:
            self._history.key_right(False)
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
        if key_event.key() == Qt.Key_Left:
            return self._handle_left_key(key_event)
        if key_event.key() == Qt.Key_Right:
            return self._handle_right_key(key_event)

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

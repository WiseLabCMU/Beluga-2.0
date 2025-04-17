from PyQt5.QtWidgets import QWidget, QPlainTextEdit, QApplication
from PyQt5.QtCore import QObject, QEvent, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QKeyEvent, QKeySequence, QTextCursor
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
                                           self._current_displayed_str[self._cursor_position:])
        self._cursor_position += len(c)

        if live_text:
            self._current_str = self._current_displayed_str

    def backspace(self, remove_word: bool, delete: bool = False):
        live_text = not self._using_history()

        if not self._current_displayed_str.strip():
            return

        if remove_word:
            part1: str = self._current_displayed_str[:self._cursor_position]
            part2: str | list[str] = self._current_displayed_str[self._cursor_position:]
            if not delete:
                part1 = part1.rstrip().rsplit(' ', 1)[0]
                self._cursor_position = len(part1)
                if part1:
                    part1 += ' '
            else:
                part2 = part2.lstrip().split(' ', 1)
                if len(part2) > 1:
                    part2 = ' ' + part2[1]
                else:
                    part2 = ''

            self._current_displayed_str = part1 + part2
        else:
            if not delete:
                if self._cursor_position > 0:
                    self._cursor_position -= 1
                else:
                    return
            self._current_displayed_str = (self._current_displayed_str[:self._cursor_position] +
                                            self._current_displayed_str[self._cursor_position + 1:])

        if live_text:
            self._current_str = self._current_displayed_str

    def home(self):
        self._cursor_position = 0

    def end(self):
        self._cursor_position = len(self._current_displayed_str)



class BelugaTerminal(QPlainTextEdit):
    enter_pressed = pyqtSignal()
    _maximum_command_storage = 128

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.installEventFilter(self)
        self._slider_shown = False
        self._dat_out_updated = False
        self._history = CommandHistory()
        self._open = False
        self._context_menu_open = False

        self._display_text: str = ""
        self._data_in = ""

        self._default_format = self.textCursor().charFormat()


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

    def _handle_up_key(self, event: QKeyEvent):
        if not (event.modifiers() & Qt.ShiftModifier):
            self._history.key_up()
            self.update_display()
        return True

    def _handle_down_key(self, event: QKeyEvent):
        if not (event.modifiers() & Qt.ShiftModifier):
            self._history.key_down()
            self.update_display()
        return True

    def _handle_enter_pressed(self, event: QKeyEvent):
        # if not self._open:
        #     return True
        if not (event.modifiers() & Qt.ControlModifier) and not (event.modifiers() & Qt.ShiftModifier):
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

    def _handle_home(self, event: QKeyEvent):
        if not (event.modifiers() & Qt.ControlModifier):
            self._history.home()
            self.update_cursor()
        return True

    def _handle_end(self, event: QKeyEvent):
        if not (event.modifiers() & Qt.ControlModifier):
            self._history.end()
            self.update_cursor()
        return True

    def _handle_delete(self, event: QKeyEvent):
        if event.modifiers() & Qt.ControlModifier:
            self._history.backspace(True, True)
        else:
            self._history.backspace(False, True)
        self.update_display()
        self.update_cursor()
        return True

    def _handle_character(self, event: QKeyEvent):
        if event.modifiers() & Qt.ControlModifier:
            return
        self._history.append_char(event.text())

    def _handle_key_press_line_edit(self, key_event: QKeyEvent):
        match(key_event.key()):
            case Qt.Key_Up:
                return self._handle_up_key(key_event)
            case Qt.Key_Down:
                return self._handle_down_key(key_event)
            case Qt.Key_Return | Qt.Key_Enter:
                return self._handle_enter_pressed(key_event)
            case Qt.Key_Backspace:
                return self._handle_backspace(key_event)
            case Qt.Key_Left:
                return self._handle_left_key(key_event)
            case Qt.Key_Right:
                return self._handle_right_key(key_event)
            case Qt.Key_Home:
                return self._handle_home(key_event)
            case Qt.Key_End:
                return self._handle_end(key_event)
            case Qt.Key_Delete:
                return self._handle_delete(key_event)
            case Qt.Key_Escape | Qt.Key_Backtab | Qt.Key_Insert | Qt.Key_Pause | Qt.Key_Print | Qt.Key_SysReq \
                 | Qt.Key_Clear | Qt.Key_Shift | Qt.Key_Control | Qt.Key_Meta | Qt.Key_Alt | Qt.Key_AltGr \
                | Qt.Key_CapsLock | Qt.Key_NumLock | Qt.Key_ScrollLock:
                pass
            case _:
                self._handle_character(key_event)


    def _handle_key_press(self, target, event: QEvent):
        key_event = QKeyEvent(event)
        if (key_event.modifiers() & Qt.ControlModifier) == Qt.ControlModifier:
            # Check if this is a shortcut for cut
            if QKeySequence(key_event.key() | Qt.ControlModifier) == QKeySequence.Cut:
                QApplication.clipboard().setText(self.textCursor().selection().toPlainText())
                return True
        self._dat_out_updated = True
        return self._handle_key_press_line_edit(key_event)

    def eventFilter(self, target: QObject, event: QEvent):
        if target == self.verticalScrollBar():
            self._handle_vertical_scroll_bar_event(event)
        elif event.type() == QEvent.KeyPress:
            ret = self._handle_key_press(target, event)
            if ret is not None:
                return ret

        return QObject().eventFilter(target, event)

    def _handle_text_select_display(self) -> (int, int, Optional[QTextCursor], bool, bool, int):
        ui_anchor = 0
        ui_pos = 0
        cursor = None
        shift_start = False
        shift_end = False
        ui_current_size = 0
        if self.textCursor().anchor() != self.textCursor().position():
            # Text selected
            ui_anchor = self.textCursor().anchor()
            ui_pos = self.textCursor().position()
            cursor = self.textCursor()
            if ui_anchor >= len(self._display_text):
                # Start of selected text is in the output buffer
                shift_start = True
                ui_current_size = len(self._display_text)
            if ui_pos >= len(self._display_text):
                # End of selected text is in the output buffer
                shift_end = True
                ui_current_size = len(self._display_text)

        return ui_anchor, ui_pos, cursor, shift_start, shift_end, ui_current_size

    def _handle_slider_display(self) -> int:
        # Slider not held down, update
        if self.verticalScrollBar().sliderPosition() == self.verticalScrollBar().maximum():
            # Scroll to bottom
            return 65535
        # Else Stay here...
        return self.verticalScrollBar().sliderPosition()

    def _display_process_input(self):
        result = []
        for c in self._data_in:
            current = ord(c)
            if current < 0x08 or (0x0B <= current <= 0x0C) or (0x0E <= current <= 0x0F):
                result.append(f"\\0{hex(current)[2:]}")
            elif (0x10 <= current <= 0x1A) or (0x1C <= current <= 0x1F):
                result.append(f"\\{hex(current)[2:]}")
            else:
                result.append(c)
        self._data_in = "".join(result)

    def _handle_append_data(self):
        if not self._data_in:
            return
        cannot_parse_bytes = 0
        unicode_replacement_char = "\ufffd"
        append_data = self._data_in
        end = 0

        i = len(append_data) - 1
        while i >= 0 and append_data[i] == unicode_replacement_char and end < 3:
            end += 1
            i -= 1

        if end > 0:
            cannot_parse_bytes += end
            append_data = append_data[:-end]

        if append_data:
            cursor = self.textCursor()
            l = 0

    def update_display(self):
        if self.verticalScrollBar().isSliderDown() or self._context_menu_open:
            return
        removed_size = 0

        ui_anchor, ui_pos, cursor, shift_start, shift_end, ui_current_size = self._handle_text_select_display()
        pos = self._handle_slider_display()
        self.setUpdatesEnabled(False)
        self._display_process_input()

        if self._data_in:
            pass

    def update_cursor(self):
        cursor = self.textCursor()
        cursor.setPosition(len(self._display_text) + self._history.cursor_position)
        cursor.setCharFormat(self._default_format)
        self.setTextCursor(cursor)

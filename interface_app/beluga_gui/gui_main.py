import sys
from PyQt5.QtWidgets import QMainWindow, QApplication
from beluga_gui import Ui_BelugaGUI


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = QMainWindow()
    ui = Ui_BelugaGUI()
    ui.setupUi(window)
    ui.comboBox.update_device_list(["test1", "test2"])

    window.show()
    sys.exit(app.exec_())
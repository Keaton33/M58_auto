import json
import sys
import threading

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QDialog
from UI import settings_ui


class SettingWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = settings_ui.Ui_SettingWindow()
        self.ui.setupUi(self)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    setting = SettingWindow()
    setting.show()
    setting.ui.pushButton_top.clicked.connect(lambda: setting.show_hoist_height_top('555'))
    sys.exit(app.exec_())

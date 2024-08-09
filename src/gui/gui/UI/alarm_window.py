import sys

from PyQt5.QtWidgets import QApplication, QDialog
from UI import alarm_ui


class AlarmWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = alarm_ui.Ui_AlarmWindow()
        self.ui.setupUi(self)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    alarm = AlarmWindow()
    alarm.show()
    sys.exit(app.exec_())

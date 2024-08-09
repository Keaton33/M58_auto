import sys

from PyQt5.QtWidgets import QApplication, QDialog
from UI import comm_ui


class CommWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = comm_ui.Ui_CommWindow()
        self.ui.setupUi(self)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    setting = CommWindow()
    setting.show()
    sys.exit(app.exec_())
